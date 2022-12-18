#!/usr/bin/python
# -*- coding: utf-8 -*-

from typing import List
import rospy
import random
import time
from std_srvs.srv import Empty
import controller_manager_msgs.srv
import geometry_msgs.msg
from nav_msgs.msg import Odometry
from gazebo_msgs.srv import GetModelState, SetModelState
from gazebo_msgs.msg import ModelState
from controller_manager_msgs.srv import (
    ListControllers,
    ReloadControllerLibraries,
    ReloadControllerLibrariesRequest,
    ReloadControllerLibrariesResponse,
)
from controller_manager_msgs.msg import ControllerState

# HSR 強化学習用 Gazebo上での反復試行テスト


EPISODE_NUM = 100
MAX_STEPS = 10

current_odom_pos: Odometry = None


pub = rospy.Publisher("/hsrb/command_velocity", geometry_msgs.msg.Twist, queue_size=10)

sim_reset = rospy.ServiceProxy("/gazebo/reset_simulation", Empty)
world_reset = rospy.ServiceProxy("/gazebo/reset_world", Empty)
pause = rospy.ServiceProxy("/gazebo/pause_physics", Empty)
unpause = rospy.ServiceProxy("/gazebo/unpause_physics", Empty)
get_model_state = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)
set_model_state = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)
list_controllers = rospy.ServiceProxy(
    "/hsrb/controller_manager/list_controllers", ListControllers
)
reload_controllers = rospy.ServiceProxy(
    "/hsrb/controller_manager/reload_controller_libraries", ReloadControllerLibraries
)


def reset():
    rospy.loginfo("Env reset")
    # Twistの値を0にする
    pub.publish(geometry_msgs.msg.Twist())

    # Gazeboをリセット
    reset_world()

    set_hsrb_state()

    # reset_controllers()


def step(action):
    # rospy.loginfo("Env step")

    # odom_data: Odometry = NoneM
    # while odom_data is None:
    #     try:
    #         odom_data = rospy.wait_for_message(
    #             '/hsrb/odom', Odometry, timeout=5)
    #     except:
    #         pass
    # current_pos = odom_data.pose.pose.position

    # if current_odom_pos is not None:
    #     rospy.loginfo(
    #         f"Current Odom Position: x={current_odom_pos.x: .2f} y={current_odom_pos.y: .2f}")

    hsrb_state: ModelState = get_hsrb_state()
    if hsrb_state is not None:
        current_model_pos = hsrb_state.pose.position
        rospy.loginfo(
            f"Current Model Position: x={current_model_pos.x: .2f} y={current_model_pos.y: .2f}"
        )

    # fill ROS message
    tw = geometry_msgs.msg.Twist()

    if action == 0:  # FORWARD
        rospy.loginfo("MOVE FORWARD")
        tw.linear.x = 10.0
        tw.angular.z = 0.0
    elif action == 1:  # LEFT
        rospy.loginfo("MOVE LEFT")
        tw.linear.x = 5.0
        tw.angular.z = 10.0
    elif action == 2:  # RIGHT
        rospy.loginfo("MOVE RIGHT")
        tw.linear.x = 0.0
        tw.angular.z = -10.0
    elif action == 3:  # BACKWARD
        rospy.loginfo("MOVE BACKWARD")
        tw.linear.x = -10.0
        tw.angular.z = 0.0

    # publish ROS message
    pub.publish(tw)


def odom_callback(msg: Odometry):
    global current_odom_pos
    pos = msg.pose.pose.position
    current_odom_pos = pos


def reset_simulation():
    rospy.wait_for_service("/gazebo/reset_simulation")
    try:
        sim_reset()
        rospy.loginfo("Reset gazebo simulation")
    except rospy.ServiceException as e:
        rospy.logerr(f"/gazebo/reset_simulation service call failed: {e}")


def reset_world():
    rospy.wait_for_service("/gazebo/reset_world")
    try:
        world_reset()
        rospy.loginfo("Reset gazebo world")
    except rospy.ServiceException as e:
        rospy.logerr(f"/gazebo/reset_world service call failed: {e}")


def pause_physics():
    rospy.wait_for_service("/gazebo/pause_physics")
    try:
        pause()
        rospy.loginfo("Pause physics")
    except rospy.ServiceException as e:
        print("/gazebo/pause_physics service call failed")


def unpause_physics():
    rospy.wait_for_service("/gazebo/unpause_physics")
    try:
        unpause()
        rospy.loginfo("Unpause physics")
    except rospy.ServiceException as e:
        print("/gazebo/unpause_physics service call failed")


def get_hsrb_state():
    rospy.wait_for_service("/gazebo/get_model_state")
    try:
        state = get_model_state("hsrb", None)
        # rospy.loginfo("Get model state")
        return state
    except rospy.ServiceException as e:
        print("/gazebo/get_model_state service call failed")


def set_hsrb_state():
    state = ModelState()
    state.model_name = "hsrb"
    state.pose.position.x = 0
    state.pose.position.y = 0
    state.pose.position.z = 0
    state.pose.orientation.x = 0
    state.pose.orientation.y = 0
    state.pose.orientation.z = 0
    state.pose.orientation.w = 1
    state.reference_frame = "world"
    rospy.wait_for_service("/gazebo/set_model_state")
    try:
        set_model_state(state)
        rospy.loginfo("Init model state")
    except rospy.ServiceException as e:
        print("/gazebo/set_model_state service call failed")

    rospy.wait_for_service("/hsrb/controller_manager/list_controllers")
    try:
        set_model_state(state)
        rospy.loginfo("Init model state")
    except rospy.ServiceException as e:
        print("/gazebo/set_model_state service call failed")


def reset_controllers():
    rospy.wait_for_service("/hsrb/controller_manager/reload_controller_libraries")
    try:
        response: ReloadControllerLibrariesResponse = reload_controllers()
        rospy.loginfo(f"Reload controllers: {response.ok}")
    except rospy.ServiceException as e:
        print(
            "/hsrb/controller_manager/reload_controller_libraries service call failed"
        )


def start_node():
    rospy.init_node("hsr_iteration")
    rospy.loginfo("Node started")

    rospy.Subscriber("/hsrb/odom", Odometry, odom_callback)

    while pub.get_num_connections() == 0:
        rospy.sleep(0.1)
    # make sure the controller is running
    rospy.wait_for_service("/hsrb/controller_manager/list_controllers")
    running = False
    while running is False:
        rospy.sleep(0.1)
        for c in list_controllers().controller:
            if c.name == "omni_base_controller" and c.state == "running":
                running = True

    run()


def run():
    for episode in range(EPISODE_NUM):

        rospy.loginfo(f"============ Start Episode {episode} ============")

        reset()
        time.sleep(0)

        for timestep in range(MAX_STEPS):
            rospy.loginfo(f"Episode: {episode}, Timestep: {timestep}")
            action = random.randint(0, 2)
            step(action)
            time.sleep(0.3)

    rospy.loginfo("Done.")


if __name__ == "__main__":
    try:
        start_node()
    except rospy.ROSInterruptException:
        pass
