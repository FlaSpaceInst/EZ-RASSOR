#!/usr/bin/env python
import rospy
from std_msgs.msg import Int8, Int16, String
from gazebo_msgs.msg import LinkStates
from sensor_msgs.msg import JointState
from ai_control.msg import ObstacleDetection
import time
import math

# Constants
RATE = 30
ARM_RANGE = (-2.5, 2.5)
DataInitialized = 0

# ROS Node Init Parameters
command_pub = rospy.Publisher('ez_main_topic', Int16, queue_size=100)
status_pub = rospy.Publisher('ez_rassor_status', ObstacleDetection, queue_size=100)

rospy.init_node('ai_control_node', anonymous=True)
rate = rospy.Rate(RATE) # 30hz

# Robot Command Dictionary
commands = {'forward' : 0b100000000000, 'reverse' : 0b010000000000, 'left' : 0b001000000000, 'right' : 0b000100000000, 
                'front_arm_up' : 0b000010000000, 'front_arm_down' : 0b000001000000, 'back_arm_up' : 0b000000100000, 'back_arm_down' : 0b000000010000,
                'front_dig' : 0b000000001000, 'front_dump' : 0b000000000100, 'back_dig' : 0b000000000010, 'back_dump' : 0b000000000001,
                'arms_up' : 0b000010100000, 'arms_down' : 0b000001010000, 'null': 0b000000000000}

# Global World State Dictionary
world_state = {'positionX': 0, 'positionY': 0, 'positionZ': 0, 'front_arm_angle': 0, 'back_arm_angle': 0, 'front_arm_angle': 0, 'heading': 0, 'warning_flag': 0, 'warning_column': 'left'}

def euclidean_distance(x1, x2, y1, y2):
    """ Calculate Euclidean distance from (x1,y1) to (x2,y2). """
    return math.sqrt( (x2-x1)**2 + (y2-y1)**2 )

def jointCallBack(data):
    """ Set world_state joint position data. """
    global world_state, DataInitialized

    print(-data.position[1], data.position[0])
    world_state['front_arm_angle'] = -(data.position[1])
    world_state['back_arm_angle'] = data.position[0]
    DataInitialized = 1

# This will be replaced to capture data from visual odometry once it is working.
def linkCallBack(data):
    """ Set world_state world position data. """
    global world_state, DataInitialized

    world_state['positionX'] = data.pose[1].position.x
    world_state['positionY'] = data.pose[1].position.y
    world_state['heading'] = 0
    DataInitialized = 1

def visionCallBack(data):
    """ Set world_state vision data. """
    global world_state, DataInitialized

    world_state['warning_flag'] = data.warning_flag
    world_state['warning_column'] = data.warning_column 

def auto_forward(distance):
    """ Travel forward distance meters in a straight line. Avoid obstacles while maintaining heading. """

    start_pos = (world_state['positionX'], world_state['positionY'])
    
    while euclidean_distance(start_pos[0], world_state['positionX'], start_pos[1], world_state['positionY']) < distance:
        command_pub.publish(commands['forward'])
        rate.sleep()

    command_pub.publish(commands['null'])


def auto_reverse(distance):
    """ Travel reverse distance meters in a straight line. Avoid obstacles while maintaining heading. """

    start_pos = (world_state['positionX'], world_state['positionY'])
    
    while euclidean_distance(start_pos[0], world_state['positionX'], start_pos[1], world_state['positionY']) < distance:
        command_pub.publish(commands['reverse'])
        rate.sleep()

    command_pub.publish(commands['null'])

def auto_dig(duration):
    """ Rotate both drums inward and drive forward for duration time in seconds. """

    for i in range(duration*RATE):
        command_pub.publish(commands['forward'] | commands['front_dig'] | commands['back_dig'])
        rate.sleep()

    command_pub.publish(commands['null'])


def set_front_arm_angle(target_angle):
    """ Set front arm to absolute angle target_angle in radians. """

    global world_state

    print("Target Angle: {}\tCurrent Angle: {}".format(target_angle, world_state['front_arm_angle']))    
    if target_angle > world_state['front_arm_angle']:
        print("Going Up")
        while(target_angle > world_state['front_arm_angle']):
            command_pub.publish(commands['front_arm_up'])
            rate.sleep()
    else:
        print("Going Down")
        while(target_angle < world_state['front_arm_angle']):
            command_pub.publish(commands['front_arm_down'])
            rate.sleep()

    
    command_pub.publish(commands['null'])


def set_back_arm_angle(target_angle):
    """ Set back arm to absolute angle target_angle in radians. """

    if target_angle > world_state['back_arm_angle']:
        print("Going Up")
        while(target_angle > world_state['back_arm_angle']):
            command_pub.publish(commands['back_arm_up'])
            rate.sleep()
    else:
        print("Going Down")
        while(target_angle < world_state['back_arm_angle']):
            command_pub.publish(commands['back_arm_down'])
            rate.sleep()


    
    command_pub.publish(commands['null'])


def ai_control():
    """
    
    """

    rospy.Subscriber('gazebo/link_states', LinkStates, linkCallBack)
    rospy.Subscriber('ez_rassor/joint_states', JointState, jointCallBack)
    #rospy.Subscriber('ez_rassor/obstacles', Int16, visionCallBack)

    while DataInitialized == 0:
        abs(0)

    set_back_arm_angle(.6)
    set_front_arm_angle(.6)
    go_forward(5)
    set_back_arm_angle(-.15)
    set_front_arm_angle(-.15)
    go_dig(10)
    set_back_arm_angle(.1)
    set_front_arm_angle(.1)
    go_reverse(10)


if __name__ == "__main__":
    try:
        ai_control()
    except rospy.ROSInterruptException:
        pass
