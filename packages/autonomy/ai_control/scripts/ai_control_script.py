#!/usr/bin/env python
import rospy
from std_msgs.msg import Int8, Int16, String
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import LinkStates
from sensor_msgs.msg import JointState
from WorldState import WorldState
from ai_config import *
from auto_functions import *
from nav_functions import *


def set_front_arm_angle(world_state, target_angle):
    """ Set front arm to absolute angle target_angle in radians. """

    if target_angle > world_state.state_flags['front_arm_angle']:
        while(target_angle > world_state.state_flags['front_arm_angle']):
            command_pub.publish(commands['front_arm_up'])
            rate.sleep()
    else:
        while(target_angle < world_state.state_flags['front_arm_angle']):
            command_pub.publish(commands['front_arm_down'])
            rate.sleep()

    
    command_pub.publish(commands['null'])


def set_back_arm_angle(world_state, target_angle):
    """ Set back arm to absolute angle target_angle in radians. """

    if target_angle > world_state.state_flags['back_arm_angle']:
        while(target_angle > world_state.state_flags['back_arm_angle']):
            command_pub.publish(commands['back_arm_up'])
            rate.sleep()
    else:
        while(target_angle < world_state.state_flags['back_arm_angle']):
            command_pub.publish(commands['back_arm_down'])
            rate.sleep()

    command_pub.publish(commands['null'])

def onStartUp():
    world_state = WorldState()

    rospy.Subscriber('stereo_odometer/odometry', Odometry, world_state.odometryCallBack)
    rospy.Subscriber('ez_rassor/joint_states', JointState, world_state.jointCallBack)
    rospy.Subscriber('ez_rassor/obstacle_detect', Int8, world_state.visionCallBack)
    rospy.Subscriber('/ezrassor/routine_toggles', Int8, world_state.autoCommandCallBack)

    set_back_arm_angle(world_state, .785)
    set_front_arm_angle(world_state, .785)

    return world_state

def ai_control(world_state):
    """ Control Auto Functions based on auto_function_command input. """

    while(True):

        while world_state.auto_function_command == 0:
            rate.sleep()

        if world_state.auto_function_command == 1:
            auto_drive(world_state)
        elif world_state.auto_function_command == 2:
            auto_dig(world_state, 10)
        elif world_state.auto_function_command == 3:
            world_state.auto_dock(world_state)
        else:
            status_pub.publish("Error Incorrect Auto Function Request {}".format(auto_function_command))


if __name__ == "__main__":
    try:
        # ROS Node Init Parameters      # ezrassor/routine_responses
        command_pub = rospy.Publisher('ez_main_topic', Int16, queue_size=100)
        status_pub = rospy.Publisher('ez_rassor/status', String, queue_size=100)
        rospy.init_node('ai_control_node', anonymous=True)
        rate = rospy.Rate(RATE) # 30hz
        
        world_state = onStartUp()
        ai_control(world_state)

    except rospy.ROSInterruptException:
        pass
