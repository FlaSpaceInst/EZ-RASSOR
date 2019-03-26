#!/usr/bin/env python
"""A ROS node that handles IMU data to start a self right sequence.

Written by Harrison Black.
Part of the EZ-RASSOR suite of software.
"""
import rospy
import std_msgs
from std_msgs.msg import Float64
from sensor_msgs.msg import Imu
import time
import sys

time_on_side_start = 0
time_on_side_start_found = False

NODE = "self_right"
TOPIC = "/imu"
WORLD = sys.argv[1]

pub_FA = rospy.Publisher("/ez_rassor/arm_front_velocity_controller/command", Float64, queue_size=10)
pub_BA = rospy.Publisher("/ez_rassor/arm_back_velocity_controller/command", Float64, queue_size=10)

def self_right_from_side():
    """ Flip EZ-RASSOR over from its side. """

    self_right_execution_time = 0.5
    arm_speed = 1
    start_time = time.time()
    print "Initiating Self Right"
    while(time.time() - start_time < self_right_execution_time):
        pub_FA.publish(-arm_speed)
        pub_BA.publish(arm_speed)

    pub_FA.publish(0)
    pub_BA.publish(0)

def parse_imu(instruction):
    """ Parse IMU data to determine if
        the EZ-RASSOR is on its side. """

    global time_on_side_start
    global time_on_side_start_found
    on_side_threshold = 1.4 if (WORLD == "moon.world") else 9
    time_on_side_limit = 6
    current_time = time.time()

    if abs(instruction.linear_acceleration.y) > on_side_threshold:
        if not time_on_side_start_found:
            time_on_side_start = time.time()
            time_on_side_start_found = True
      
        time_on_side_total = time.time() - time_on_side_start
        if(time_on_side_total > time_on_side_limit):
            self_right_from_side()

    else:
        time_on_side_start = 0;
        time_on_side_start_found = False


# Main entry point to the node.
try:
    print("Self right started")
    rospy.init_node(NODE, anonymous=True)
    rospy.Subscriber(TOPIC, Imu, parse_imu)
    rospy.spin()
except rospy.ROSInterruptException:
    pass