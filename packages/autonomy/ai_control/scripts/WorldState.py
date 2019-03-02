#!/usr/bin/env python
import rospy
from std_msgs.msg import Int8, Int16, String
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import LinkStates
from sensor_msgs.msg import JointState
from ai_control.msg import ObstacleDetection
import time
import math


class WorldState():

    def __init__(self):
        self.state_flags = {'positionX': 0, 'positionY': 0, 'positionZ': 0, 
                            'front_arm_angle': 0, 'back_arm_angle': 0, 
                            'front_arm_angle': 0, 'heading': 0, 'warning_flag': 0}

        self. auto_function_command = 0


    def jointCallBack(self, data):
        """ Set state_flags joint position data. """

        self.state_flags['front_arm_angle'] = -(data.position[1])
        self.state_flags['back_arm_angle'] = data.position[0]
    

    def odometryCallBack(self, data):
        """ Set state_flags world position data. """

        self.state_flags['positionX'] = data.pose.pose.position.z
        self.state_flags['positionY'] = data.pose.pose.position.y
        self.state_flags['heading'] = data.twist.twist.linear.z
        

    def visionCallBack(self, data):
        """ Set state_flags vision data. """

        self.state_flags['warning_flag'] = data.data

    def autoCommandCallBack(self, data):
        """ Set auto_function_command to the current choice. """
        self.auto_function_command = data.data



if __name__ == "__main__":
    state_flags = WorldState()

    rospy.Subscriber('stereo_odometer/odometry', Odometry, state_flags.odometryCallBack)
    rospy.Subscriber('ez_rassor/joint_states', JointState, state_flags.jointCallBack)
    rospy.Subscriber('ez_rassor/obstacle_detect', Int8, state_flags.visionCallBack)
    rospy.Subscriber('/ezrassor/routine_toggles', Int8, state_flags.autoCommandCallBack)

