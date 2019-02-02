#!/usr/bin/env python
"""A ROS node that moves the arms on the EZRC.

Written by Tiger Sachse.
Part of the EZ-RASSOR suite of software.
"""
import rospy
import std_msgs
from std_msgs.msg import Int16, Float64

NODE = "arms"
TOPIC = "ez_main_topic"
MASK = 0b000011110000
MESSAGE_FORMAT = "EZRC (arms.py): %s."

# /ez_rassor/arm_back_velocity_controller/command
pub_FA = rospy.Publisher('/ez_rassor/arm_front_velocity_controller/command', Float64, queue_size = 10)
pub_BA = rospy.Publisher('/ez_rassor/arm_back_velocity_controller/command', Float64, queue_size = 10)


def get_movements(integer, mask):
    """Decode a bitstring to reveal the movement commands for this node."""

    # Use a mask to remove unnecessary bits.
    result = integer & mask

    # Shift the result so the commands are the 4 least significant bits. The
    # amount of shifting is determined by the mask.
    while mask % 2 == 0:
        result >>= 1
        mask >>= 1

    # Return the 4 least significant bits as boolean values.
    return (
        result & 0b1000 != 0,
        result & 0b100 != 0,
        result & 0b10 != 0,
        result & 0b1 != 0
    )


def handle_arm_movements(instruction):
    """Move the arms of the EZRC per the commands encoded in the instruction."""
    arm1_up, arm1_down, arm2_up, arm2_down = get_movements(instruction.data, MASK)

    arm_speed = 1

    if not any((arm1_up, arm1_down, arm2_up, arm2_down)):
        # print MESSAGE_FORMAT % "Stopping both arms"
        # stop both arms
        pub_FA.publish(0)
        pub_BA.publish(0)

    else:

        # The arms are mirrored and the model so arm1 needs negative velocity to move up.
        if arm1_up:
            # print MESSAGE_FORMAT % "Raising arm 1"
            # raise arm 1
            pub_FA.publish(-arm_speed)

        if arm1_down:
            # print MESSAGE_FORMAT % "Lowering arm 1"
            # lower arm 1
            pub_FA.publish(arm_speed)


        if arm2_up:
            # print MESSAGE_FORMAT % "Raising arm 2"
            # raise arm 2
            pub_BA.publish(arm_speed)

        if arm2_down:
            # print MESSAGE_FORMAT % "Lowering arm 2"
            # lower arm 2
            pub_BA.publish(-arm_speed)



# Main entry point to the node.
try:
    print("Arms node started")
    rospy.init_node(NODE, anonymous=True)
    rospy.Subscriber(TOPIC, std_msgs.msg.Int16, handle_arm_movements)
    rospy.spin()
except rospy.ROSInterruptException:
    pass
