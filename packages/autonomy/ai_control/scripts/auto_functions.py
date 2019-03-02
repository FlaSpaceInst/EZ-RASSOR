#!/usr/bin/env python
import rospy
from std_msgs.msg import Int8, Int16, String
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import LinkStates
from sensor_msgs.msg import JointState
from ai_control.msg import ObstacleDetection
from WorldState import WorldState
import time
import math

def auto_drive(world_state):
    """ Travel forward in a straight line. Avoid obstacles while maintaining heading. """
    
    while world_state.auto_function_command != 0:
        while(world_state.state_flags['warning_flag'] == 1):
            command_pub.publish(commands['right'])
            rate.sleep()
        while(world_state.state_flags['warning_flag'] == 2):
            command_pub.publish(commands['left'])
            rate.sleep()
        
        command_pub.publish(commands['forward'])
        rate.sleep()

    command_pub.publish(commands['null'])

def auto_drive_location(world_state, location):
    """ Navigate to location. Avoid obstacles while moving toward location. """
    pass


def auto_dig(world_state, duration):
    """ Rotate both drums inward and drive forward for duration time in seconds. """

    while world_state.auto_function_command != 0:
        while(world_state.state_flags['warning_flag_front'] == 1):
            command_pub.publish(commands['right'])
            rate.sleep()
        while(world_state.state_flags['warning_flag_front'] == 2):
            command_pub.publish(commands['left'])
            rate.sleep()
        
        command_pub.publish(commands['forward'] | commands['front_dig'] | commands['back_dig'])
        rate.sleep()

    command_pub.publish(commands['null'])

def auto_dock():
    """ Dock with the hopper. """
    pass