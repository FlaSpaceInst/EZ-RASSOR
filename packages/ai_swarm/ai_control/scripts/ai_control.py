#!/usr/bin/env python
import rospy
from std_msgs.msg import Int8, Int16
from gazebo_msgs.msg import LinkStates
import time
import math

# Constants
RATE = 30
DIG_DURATION = 5

# Global State Variables
posx = 0
posy = 0
heading = 0
front_arm_angle = 0
back_arm_angle = 0


# ROS Node Init Parameters
pub = rospy.Publisher('ez_main_topic', Int16, queue_size=100)
rospy.init_node('ai_control_node', anonymous=True)
rate = rospy.Rate(RATE) # 600hz

# Robot Command Dictionary
commands = {'forward' : 0b100000000000, 'reverse' : 0b010000000000, 'left' : 0b001000000000, 'right' : 0b000100000000, 
                'front_arm_up' : 0b000010000000, 'front_arm_down' : 0b000001000000, 'back_arm_up' : 0b000000100000, 'back_arm_down' : 0b000000010000,
                'front_dig' : 0b000000001000, 'front_dump' : 0b000000000100, 'back_dig' : 0b0000000000100, 'back_dump' : 0b000000000001,
                'arms_up' : 0b000010100000, 'arms_down' : 0b000001010000}

# Pre-Planned Path and Index           
path = [("forward", 5), ("left", 90), ("forward", 5), ("left", 90), ("forward", 5), ("left", 90), ("forward", 5), ("left", 90)]
path_stage = 0

def jointCallBack(data):
    print(data.position[2])

def linkCallBack(data):
    global posx, posy, heading, front_arm_angle, back_arm_angle

    posx = data.pose[1].position.x
    posy = data.pose[1].position.y
    heading = data.pose[1].orientation.z
    front_arm_angle = data.pose

def go_forward(distance):
    print('Going Forward')
    global posx, posy
    start_pos = (posx, posy)

    while euclidean_distance(start_pos[0], posx, start_pos[1], posy) < distance:
        pub.publish(commands['forward'])
        rate.sleep()

    pub.publish(0b000000000000)


def go_reverse(distance):
    print("Reversing")
    global posx, posy
    start_pos = (posx, posy)

    while euclidean_distance(start_pos[0], posx, start_pos[1], posy) < distance:
        pub.publish(commands['reverse'])
        rate.sleep()

    pub.publish(0b000000000000)

def go_dig(duration):
    print('Currently Digging')
    for i in range(duration*RATE):
        if i % RATE == 0:
            print(i / RATE)
        rate.sleep()

    pub.publish(0b000000000000)


def set_arm_angle(duration, command):
    print('Adjusting Arms')
    for i in range(duration*RATE):
        if i % RATE == 0:
            print(i / RATE)
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
    go_reverse(10)
        

if __name__ == "__main__":
    try:
        ai_control()
    except rospy.ROSInterruptException:
        pass
