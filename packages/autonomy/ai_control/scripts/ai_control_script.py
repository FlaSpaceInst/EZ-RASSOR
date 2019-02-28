#!/usr/bin/env python
import rospy
from std_msgs.msg import Int8, Int16, String
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import LinkStates
from sensor_msgs.msg import JointState
from ai_control.msg import ObstacleDetection
import time
import math

# Constants
RATE = 30
ARM_RANGE = (-2.5, 2.5)
auto_function_command = 1

# ROS Node Init Parameters      # ezrassor/routine_responses
command_pub = rospy.Publisher('ez_main_topic', Int16, queue_size=100)
status_pub = rospy.Publisher('ez_rassor/status', String, queue_size=100)

rospy.init_node('ai_control_node', anonymous=True)
rate = rospy.Rate(RATE) # 30hz

# Robot Command Dictionary
commands = {'forward' : 0b100000000000, 'reverse' : 0b010000000000, 'left' : 0b001000000000, 'right' : 0b000100000000, 
                'front_arm_up' : 0b000010000000, 'front_arm_down' : 0b000001000000, 'back_arm_up' : 0b000000100000, 'back_arm_down' : 0b000000010000,
                'front_dig' : 0b000000001000, 'front_dump' : 0b000000000100, 'back_dig' : 0b000000000010, 'back_dump' : 0b000000000001,
                'arms_up' : 0b000010100000, 'arms_down' : 0b000001010000, 'null': 0b000000000000}

# Global World State Dictionary
world_state = {'positionX': 0, 'positionY': 0, 'positionZ': 0, 'front_arm_angle': 0, 'back_arm_angle': 0, 'front_arm_angle': 0, 'heading': 0, 'warning_flag': 0}

def euclidean_distance(x1, x2, y1, y2):
    """ Calculate Euclidean distance from (x1,y1) to (x2,y2). """
    return math.sqrt( (x2-x1)**2 + (y2-y1)**2 )

def jointCallBack(data):
    """ Set world_state joint position data. """
    global world_state

    world_state['front_arm_angle'] = -(data.position[1])
    world_state['back_arm_angle'] = data.position[0]
    

# This will be replaced to capture data from visual odometry once it is working.
def odometryCallBack(data):
    """ Set world_state world position data. """
    global world_state

    world_state['positionX'] = data.pose.pose.position.z
    world_state['positionY'] = data.pose.pose.position.y
    world_state['heading'] = data.twist.twist.linear.z
    print(world_state['positionX'], world_state['positionY'])

def visionCallBack(data):
    """ Set world_state vision data. """
    global world_state

    world_state['warning_flag'] = data.data

def autoCommandCallBack(data):
    """ Set auto_function_command to the current choice. """
    global auto_function_command
    auto_function_command = data.data

def auto_drive():
    """ Travel forward in a straight line. Avoid obstacles while maintaining heading. """
    
    while auto_function_command != 0:
        while(world_state['warning_flag'] == 1):
            command_pub.publish(commands['right'])
            rate.sleep()
        while(world_state['warning_flag'] == 2):
            command_pub.publish(commands['left'])
            rate.sleep()
        
        command_pub.publish(commands['forward'])
        rate.sleep()

    command_pub.publish(commands['null'])

def auto_drive_location(location):
    """ Navigate to location. Avoid obstacles while moving toward location. """
    pass


def auto_reverse(distance):
    """ Travel reverse distance meters in a straight line. Avoid obstacles while maintaining heading. """
    pass


def auto_dig(duration):
    """ Rotate both drums inward and drive forward for duration time in seconds. """

    while auto_function_command != 0:
        while(world_state['warning_flag_front'] == 1):
            command_pub.publish(commands['right'])
            rate.sleep()
        while(world_state['warning_flag_front'] == 2):
            command_pub.publish(commands['left'])
            rate.sleep()
        
        command_pub.publish(commands['forward'] | commands['front_dig'] | commands['back_dig'])
        rate.sleep()

    command_pub.publish(commands['null'])

def auto_dock():
    """ Dock with the hopper. """
    pass


def set_front_arm_angle(target_angle):
    """ Set front arm to absolute angle target_angle in radians. """

    if target_angle > world_state['front_arm_angle']:
        while(target_angle > world_state['front_arm_angle']):
            command_pub.publish(commands['front_arm_up'])
            rate.sleep()
    else:
        while(target_angle < world_state['front_arm_angle']):
            command_pub.publish(commands['front_arm_down'])
            rate.sleep()

    
    command_pub.publish(commands['null'])


def set_back_arm_angle(target_angle):
    """ Set back arm to absolute angle target_angle in radians. """

    if target_angle > world_state['back_arm_angle']:
        while(target_angle > world_state['back_arm_angle']):
            command_pub.publish(commands['back_arm_up'])
            rate.sleep()
    else:
        while(target_angle < world_state['back_arm_angle']):
            command_pub.publish(commands['back_arm_down'])
            rate.sleep()


    
    command_pub.publish(commands['null'])

def onStartUp():
    set_back_arm_angle(.785)
    set_front_arm_angle(.785)

def ai_control():
    """
    
    """
    rospy.Subscriber('stereo_odometer/odometry', Odometry, odometryCallBack)
    rospy.Subscriber('ez_rassor/joint_states', JointState, jointCallBack)
    rospy.Subscriber('ez_rassor/obstacle_detect', Int8, visionCallBack)
    rospy.Subscriber('/ezrassor/routine_toggles', Int8, autoCommandCallBack)

    while(True):

        while auto_function_command == 0:
            rate.sleep()

        if auto_function_command == 1:
            auto_drive()
        elif auto_function_command == 2:
            auto_dig(10)
        elif auto_function_command == 3:
            auto_dock()
        else:
            status_pub.publish("Error Incorrect Auto Function Request {}".format(auto_function_command))


if __name__ == "__main__":
    try:
        onStartUp()
        ai_control()
    except rospy.ROSInterruptException:
        pass
