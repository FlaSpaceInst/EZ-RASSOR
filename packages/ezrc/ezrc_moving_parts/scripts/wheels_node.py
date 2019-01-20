#!/usr/bin/env python
"""A ROS node that moves the wheels on the EZRC.

Written by Tiger Sachse and Harrison Black.
Part of the EZ-RASSOR suite of software.
"""
import rospy
import std_msgs
import utilities

MASK = 0b111100000000
NODE_NAME = "wheels_node"
TOPIC_NAME = "ezmain_topic"
MESSAGE_FORMAT = "EZRC (wheels_node.py): %s."

def handle_wheel_movements(instruction):
    """Move the wheels of the EZRC per the commands encoded in the instruction."""
    wheel1_forward, wheel1_backward, wheel2_forward, wheel2_backward = utilities.get_nibble(
        instruction.data,
        MASK
    )

    if not any((wheel1_forward, wheel1_backward, wheel2_forward, wheel2_backward)):
        print MESSAGE_FORMAT % "Stopping both wheels"
        # stop both wheels
    else:
        if wheel1_forward:
            print MESSAGE_FORMAT % "Rotating wheel 1 forward"
            # rotate wheel 1 forward

        if wheel1_backward:
            print MESSAGE_FORMAT % "Rotating wheel 1 backward"
            # rotate wheel 1 backward

        if wheel2_forward:
            print MESSAGE_FORMAT % "Rotating wheel 2 forward"
            # rotate wheel 2 forward

        if wheel2_backward:
            print MESSAGE_FORMAT % "Rotating wheel 2 backward"
            # rotate wheel 2 backward

# Main entry point to the node.
try:
    rospy.init_node(NODE_NAME, anonymous=True)
    rospy.Subscriber(TOPIC_NAME, std_msgs.msg.Int16, handle_wheel_movements)
    rospy.spin()
except rospy.ROSInterruptException:
    pass
