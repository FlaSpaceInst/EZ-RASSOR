#!/usr/bin/env python
"""Translate data from joy_node into something the EZ-RASSOR can understand.

Written by Harrison Black.
"""
import rospy
from std_msgs.msg import Float32
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

NODE = "joy_translator"
TOPIC = "joy"
PUBLISH_TOPIC_WHEELS = rospy.get_param("/joy_translator/movement_topic")
PUBLISH_TOPIC_ARMS = rospy.get_param("/joy_translator/arms_topic")
PUBLISH_TOPIC_DRUMS = rospy.get_param("/joy_translator/drums_topic")
MAX_WHEEL_SPEED = rospy.get_param("/joy_translator/max_wheel_speed")
MAX_ARM_SPEED = rospy.get_param("/joy_translator/max_arm_speed")
MAX_DRUM_SPEED = rospy.get_param("/joy_translator/max_drum_speed")

# Publishers
# Wheel twist
pub_wheels = rospy.Publisher(PUBLISH_TOPIC_WHEELS,
                             Twist, 
                             queue_size=10)
# Arm Front
pub_front_arm = rospy.Publisher("front_"+PUBLISH_TOPIC_ARMS, 
	                            Float32, 
	                            queue_size=10)
# Arm Back
pub_back_arm = rospy.Publisher("back_"+PUBLISH_TOPIC_ARMS, 
	                           Float32, 
	                           queue_size=10)
# Drum Front
pub_front_drum = rospy.Publisher("front_"+PUBLISH_TOPIC_DRUMS, 
	                             Float32, 
	                             queue_size=10)
# Drum Back
pub_back_drum = rospy.Publisher("back_"+PUBLISH_TOPIC_DRUMS,
                                Float32, 
                                queue_size=10)

def callback(data):
    """Parse Joy data, publish Twist data"""

    # Raw controller input data indexes
    # data.buttons[index]
    # 0 A : Back Drum Dump
    # 1 B : Front Drum Dump
    # 2 X : Back Drum Dig
    # 3 Y : Front Drum Dig
    # 4 LB : Back Arm Up
    # 5 RB : Front Arm Up
    # 6 back : Tank Turn
    # 7 start : Supervisor Mode
    # 8 power : NA
    # 9 Button stick left : NA
    # 10 Button stick right : NA

    # data.axes[index]
    # 0 Left/Right Axis stick left : Turn Left/Right
    # 1 Up/Down Axis stick left : Move Forward/Reverse
    # 2 LT : Back Arm Down
    # 3 Left/Right Axis stick right : NA
    # 4 Up/Down Axis stick right : NA
    # 5 RT : Front Arm Down
    # 6 cross key left/right : Function 2/3
    # 7 cross key up/down : Function 1/4

    twist = Twist()
    twist.linear.x = data.axes[1] * float(MAX_WHEEL_SPEED)
    twist.linear.z = data.axes[0] * float(MAX_WHEEL_SPEED)
    trigger_threshold = 0.0

    # Use "-(1-data.axes[5])/2" not -1 for variable speed
    # Front Arm
    if data.buttons[5] > (1 - data.axes[5]) / 2:
    	command_front_arm = 1 * MAX_ARM_SPEED
    elif data.buttons[5] < (1 - data.axes[5]) / 2 and trigger_threshold > data.axes[5]:
    	command_front_arm = -1 * MAX_ARM_SPEED
    else:
    	command_front_arm = 0

    # Back Arm
    if data.buttons[4] > (1 - data.axes[2]) / 2:
    	command_back_arm = 1 * MAX_ARM_SPEED
    elif data.buttons[4] < (1 - data.axes[2]) / 2 and trigger_threshold > data.axes[2]:
    	command_back_arm = -1 * MAX_ARM_SPEED
    else:
    	command_back_arm = 0

    # Front Drum
    if data.buttons[3] > data.buttons[1]:
        command_front_drum = 1 * MAX_DRUM_SPEED
    elif data.buttons[3] < data.buttons[1]:
    	command_front_drum = -1 * MAX_DRUM_SPEED
    else:
    	command_front_drum = 0

    # Back Drum
    if data.buttons[2] > data.buttons[0]:
        command_back_drum = 1 * MAX_DRUM_SPEED
    elif data.buttons[2] < data.buttons[0]:
    	command_back_drum = -1 * MAX_DRUM_SPEED
    else:
    	command_back_drum = 0

    pub_wheels.publish(twist)
    pub_front_arm.publish(command_front_arm)
    pub_back_arm.publish(command_back_arm)
    pub_front_drum.publish(command_front_drum)
    pub_back_drum.publish(command_back_drum)
    

def start_node():
    try:
        print "Controller node started"
        rospy.init_node(NODE, anonymous = True)
        rate = rospy.Rate(600)
        rospy.Subscriber(TOPIC, Joy, callback)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass