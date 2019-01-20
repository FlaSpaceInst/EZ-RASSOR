#!/usr/bin/env python
"""A ROS node that moves the arms on the EZRC.

Written by Tiger Sachse and Harrison Black.
Part of the EZ-RASSOR suite of software.
"""
import rospy
import std_msgs
import utilities

MASK = 0b000011110000
NODE_NAME = "arms_node"
TOPIC_NAME = "ezmain_topic"
MESSAGE_FORMAT = "EZRC (arms_node.py): %s."

def handle_arm_movements(instruction):
    """Move the arms of the EZRC per the commands encoded in the instruction."""
    arm1_up, arm1_down, arm2_up, arm2_down = utilities.get_nibble(
        instruction.data,
        MASK
    )

    if not any((arm1_up, arm1_down, arm2_up, arm2_down)):
        print MESSAGE_FORMAT % "Stopping both arms"
        # stop both arms
    else:
        if arm1_up:
            print MESSAGE_FORMAT % "Raising arm 1"
            # raise arm 1

        if arm1_down:
            print MESSAGE_FORMAT % "Lowering arm 1"
            # lower arm 1

        if arm2_up:
            print MESSAGE_FORMAT % "Raising arm 2"
            # raise arm 2

        if arm2_down:
            print MESSAGE_FORMAT % "Lowering arm 2"
            # lower arm 2

# Main entry point to the node.
try:
    rospy.init_node(NODE_NAME, anonymous=True)
    rospy.Subscriber(TOPIC_NAME, std_msgs.msg.Int16, handle_arm_movements)
    rospy.spin()
except rospy.ROSInterruptException:
    pass
