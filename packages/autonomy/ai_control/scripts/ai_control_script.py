#!/usr/bin/env python
import rospy
from std_msgs.msg import Int8, Int16, String
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import LinkStates
from sensor_msgs.msg import JointState
from ai_objects import WorldState, RospyConfig
from auto_functions import *
from nav_functions import *
from utility_functions import *

# Bit String Commands
commands = {'forward' : 0b100000000000, 'reverse' : 0b010000000000, 'left' : 0b001000000000, 'right' : 0b000100000000, 
                'front_arm_up' : 0b000010000000, 'front_arm_down' : 0b000001000000, 'back_arm_up' : 0b000000100000, 'back_arm_down' : 0b000000010000,
                'front_dig' : 0b000000001000, 'front_dump' : 0b000000000100, 'back_dig' : 0b000000000010, 'back_dump' : 0b000000000001,
                'arms_up' : 0b000010100000, 'arms_down' : 0b000001010000, 'null': 0b000000000000}


def onStartUp():
    world_state = WorldState()
    rospy_config = RospyConfig()

    rospy.Subscriber('stereo_odometer/odometry', Odometry, world_state.odometryCallBack)
    rospy.Subscriber('ez_rassor/joint_states', JointState, world_state.jointCallBack)
    rospy.Subscriber('ez_rassor/obstacle_detect', Int8, world_state.visionCallBack)
    rospy.Subscriber('/ezrassor/routine_toggles', Int8, world_state.autoCommandCallBack)

    set_back_arm_angle(world_state, .785)
    set_front_arm_angle(world_state, .785)

    return world_state, rospy_config

def ai_control(world_state, rospy_config):
    """ Control Auto Functions based on auto_function_command input. """

    while(True):

        while world_state.auto_function_command == 0:
            rospy_config.rate.sleep()

        if world_state.auto_function_command == 1:
            auto_drive(world_state, rospy_config)
        elif world_state.auto_function_command == 2:
            auto_dig(world_state, rospy_config, 10)
        elif world_state.auto_function_command == 3:
            world_state.auto_dock(world_state, rospy_config)
        else:
            rospy_config.status_pub.publish("Error Incorrect Auto Function Request {}".format(world_state.auto_function_command))


if __name__ == "__main__":
    try:
        # ROS Node Init Parameters      # ezrassor/routine_responses
        rospy.init_node('ai_control_node', anonymous=True)
        
        world_state, rospy_config = onStartUp()
        ai_control(world_state, rospy_config)

    except rospy.ROSInterruptException:
        pass
