#!/usr/bin/env python
"""A ROS node that moves the drums on the EZRC.

Written by Tiger Sachse and Harrison Black.
Part of the EZ-RASSOR suite of software.
"""
import time
import rospy
import std_msgs
import utilities
import RPi.GPIO as GPIO

MASK = 0b000000001111
NODE_NAME = "drums_node"
TOPIC_NAME = "ezmain_topic"
MESSAGE_FORMAT = "EZRC (drums_node.py): %s."

FORWARD_PINS = (16, 21, 20)
ROTATIONAL_SPEED = .2

def rotate_drum(pins):
    """"""
    for pin in pins:
        GPIO.output(pin, GPIO.HIGH)
        time.sleep(ROTATIONAL_SPEED)
        GPIO.output(pin, GPIO.LOW)
        time.sleep(ROTATIONAL_SPEED/2)

def handle_drum_movements(instruction):
    """Move the drums of the EZRC per the commands encoded in the instruction."""
    drum1_dig, drum1_dump, drum2_dig, drum2_dump = utilities.get_nibble(
        instruction.data,
        MASK
    )

    if not any((drum1_dig, drum1_dump, drum2_dig, drum2_dump)):
        print MESSAGE_FORMAT % "Stopping both drums"
        # stop both drums
    else:
        if drum1_dig:
            print MESSAGE_FORMAT % "Digging with drum 1"
            rotate_drum(FORWARD_PINS)

        if drum1_dump:
            print MESSAGE_FORMAT % "Dumping from drum 1"
            # dump from drum 1

        if drum2_dig:
            print MESSAGE_FORMAT % "Digging with drum 2"
            # dig with drum 2

        if drum2_dump:
            print MESSAGE_FORMAT % "Dumping from drum 2"
            # dump from drum 2

# Main entry point to the node.
try:
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)

    for pin in FORWARD_PINS:
        GPIO.setup(pin, GPIO.OUT)

    rospy.init_node(NODE_NAME, anonymous=True)
    rospy.Subscriber(TOPIC_NAME, std_msgs.msg.Int16, handle_drum_movements)
    rospy.spin()
except rospy.ROSInterruptException:
    pass
