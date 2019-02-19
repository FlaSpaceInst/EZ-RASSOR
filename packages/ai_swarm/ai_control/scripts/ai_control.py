#!/usr/bin/env python
import rospy
from std_msgs.msg import Int8, Int16
from gazebo_msgs.msg import LinkStates
import time
import math

# Constants
RATE = 30
ARM_RANGE (-2.5, 9)

# ROS Node Init Parameters
command_pub = rospy.Publisher('ez_main_topic', Int16, queue_size=100)
status_pub = rospy.Publisher('ez_rassor_status', String, queue_size=100)

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
    global world_state

def linkCallBack(data):
    '''''
    '''''
    global world_state

    world_state['positionX'] = data.pose[1].position.x
    world_state['positionY'] = data.pose[1].position.y
    world_state['heading'] = 0


def go_forward(distance):
    '''''
    '''''
    global world_state

    start_pos = (world_state['positionX'], world_state['positionY'])
    
    while euclidean_distance(start_pos[0], world_state['position'], start_pos[1], world_state['positionY']) < distance:
        pub.publish(commands['forward'])
        rate.sleep()

    pub.publish(commands['null'])


def go_reverse(distance, world_state):
    '''''
    '''''
    global world_state

    start_pos = (world_state['positionX'], world_state['positionY'])
    
    while euclidean_distance(start_pos[0], world_state['position'], start_pos[1], world_state['positionY']) < distance:
        pub.publish(commands['reverse'])
        rate.sleep()

    pub.publish(commands['null'])

def go_dig(duration):
    '''''
    '''''
    global world_state

    for i in range(duration*RATE):
        pub.publish(commands['forward'] | commands['front_dig'] | commands['back_dig'])
        rate.sleep()

    pub.publish(command['null'])


def set_arm_angle(angle, command):
    '''''
    '''''
    global world_state

    for i in range(duration*RATE):
        if i % RATE == 0:
        pub.publish(commands[command])
        rate.sleep()

    pub.publish(0b000000000000)



def euclidean_distance(x1, x2, y1, y2):
    return math.sqrt( (x2-x1)**2 + (y2-y1)**2 )

def ai_control():
    global pub
    rospy.Subscriber('gazebo/link_states', LinkStates, linkCallBack)


    set_arm_angle(1, 'arms_up')
    go_forward(10)
    set_arm_angle(1, 'arms_down')
    go_dig(5)
    set_arm_angle(1, 'arms_up')
    go_reverse(15)
        

if __name__ == "__main__":
    try:
        ai_control()
    except rospy.ROSInterruptException:
        pass
