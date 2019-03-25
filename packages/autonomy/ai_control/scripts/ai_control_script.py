#!/usr/bin/env python
import rospy
import sys
from std_msgs.msg import Int8, Int16, String
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import LinkStates
from sensor_msgs.msg import JointState
from sensor_msgs.msg import Imu
from ai_objects import WorldState, ROSUtility
from auto_functions import * 
from utility_functions import *

def onStartUp():
    """  """
    print("Spinning Up AI Control")
    # ROS Node Init Parameters 
    rospy.init_node('ai_control_node', anonymous=True)
    
    #Create Utility Objects
    world_state = WorldState()
    ros_util = ROSUtility()

    ros_util.status_pub.publish("Spinning Up AI Control")

    # Setup Subscriber Callbacks
    #rospy.Subscriber('stereo_odometer/odometry', Odometry, world_state.odometryCallBack)
    rospy.Subscriber('/imu', Imu, world_state.imuCallBack)
    rospy.Subscriber('ez_rassor/joint_states', JointState, world_state.jointCallBack)
    rospy.Subscriber('ez_rassor/obstacle_detect', Int8, world_state.visionCallBack)
    rospy.Subscriber('/ezrassor/routine_toggles', Int8, ros_util.autoCommandCallBack)
    rospy.Subscriber('gazebo/link_states', LinkStates, world_state.simStateCallBack)

    set_back_arm_angle(world_state, ros_util, .785)
    set_front_arm_angle(world_state, ros_util, .785)

    return world_state, ros_util

def ai_control(world_state, ros_util):
    """ Control Auto Functions based on auto_function_command input. """

    while(True):
        # Temp
        ros_util.auto_function_command = 1

        while ros_util.auto_function_command == 0:
            ros_util.rate.sleep()

        if ros_util.auto_function_command == 1:
            auto_drive_location(world_state, ros_util)
        elif ros_util.auto_function_command == 2:
            auto_dig(world_state, ros_util, 10)
        elif ros_util.auto_function_command == 3:
            world_state.auto_dock(world_state, ros_util)
        else:
            ros_util.status_pub.publish("Error Incorrect Auto Function Request {}".format(ros_util.auto_function_command))


if __name__ == "__main__":
    world_state, ros_util = onStartUp()
    ai_control(world_state, ros_util)