#!/usr/bin/env python
import rospy
from std_msgs.msg import Int8, Int16
from gazebo_msgs.msg import LinkStates
from sensor_msgs.msg import JointState
import time
import math

# Constants
RATE = 30
ARM_RANGE = (-2.5, 2.5)
DataInitialized = 0

# ROS Node Init Parameters
command_pub = rospy.Publisher('ez_main_topic', Int16, queue_size=100)
#status_pub = rospy.Publisher('ez_rassor_status', String, queue_size=100)

rospy.init_node('ai_control_node', anonymous=True)
rate = rospy.Rate(RATE) # 30hz

# Robot Command Dictionary
commands = {'forward' : 0b100000000000, 'reverse' : 0b010000000000, 'left' : 0b001000000000, 'right' : 0b000100000000, 
                'front_arm_up' : 0b000010000000, 'front_arm_down' : 0b000001000000, 'back_arm_up' : 0b000000100000, 'back_arm_down' : 0b000000010000,
                'front_dig' : 0b000000001000, 'front_dump' : 0b000000000100, 'back_dig' : 0b000000000010, 'back_dump' : 0b000000000001,
                'arms_up' : 0b000010100000, 'arms_down' : 0b000001010000, 'null': 0b000000000000}

# Global World State Dictionary
world_state = {'positionX': 0, 'positionY': 0, 'positionZ': 0, 'front_arm_angle': 0, 'back_arm_angle': 0, 'front_arm_angle': 0, 'heading': 0}

def jointCallBack(data):
    '''''
    '''''
    global DataInitialized
    global world_state
    print(-data.position[1], data.position[0])
    world_state['front_arm_angle'] = -(data.position[1])
    world_state['back_arm_angle'] = data.position[0]
    DataInitialized = 1


def linkCallBack(data):
    '''''
    '''''
    global DataInitialized
    global world_state

    world_state['positionX'] = data.pose[1].position.x
    world_state['positionY'] = data.pose[1].position.y
    world_state['heading'] = 0
    DataInitialized = 1


def go_forward(distance):
    '''''
    '''''

    start_pos = (world_state['positionX'], world_state['positionY'])
    
    while euclidean_distance(start_pos[0], world_state['positionX'], start_pos[1], world_state['positionY']) < distance:
        command_pub.publish(commands['forward'])
        rate.sleep()

    command_pub.publish(commands['null'])


def go_reverse(distance):
    '''''
    '''''

    start_pos = (world_state['positionX'], world_state['positionY'])
    
    while euclidean_distance(start_pos[0], world_state['positionX'], start_pos[1], world_state['positionY']) < distance:
        command_pub.publish(commands['reverse'])
        rate.sleep()

    command_pub.publish(commands['null'])

def go_dig(duration):
    '''''
    '''''
    for i in range(duration*RATE):
        command_pub.publish(commands['forward'] | commands['front_dig'] | commands['back_dig'])
        rate.sleep()

    command_pub.publish(commands['null'])


def set_front_arm_angle(target_angle):
    '''''
    '''''
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
    '''''
    '''''
    print("Target Angle: {}\tCurrent Angle: {}".format(target_angle, world_state['back_arm_angle']))    
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


def euclidean_distance(x1, x2, y1, y2):
    return math.sqrt( (x2-x1)**2 + (y2-y1)**2 )

def ai_control():
    rospy.Subscriber('gazebo/link_states', LinkStates, linkCallBack)
    rospy.Subscriber('ez_rassor/joint_states', JointState, jointCallBack)

    while DataInitialized == 0:
        abs(0)

    set_back_arm_angle(.6)
    set_front_arm_angle(.6)
    go_forward(5)
    set_back_arm_angle(-.1)
    set_front_arm_angle(-.1)
    go_dig(5)
    go_reverse(10)


if __name__ == "__main__":
    try:
        ai_control()
    except rospy.ROSInterruptException:
        pass
