"""A ROS node that moves the arms on the EZRC.

Written by Harrison Black and Tiger Sachse.
Part of the EZ-RASSOR suite of software.
"""
import rospy
import std_msgs
from std_msgs.msg import Int16, Float64

NODE = "arms"
TOPIC = "movement_toggles"
MASK = 0b000011110000
MESSAGE_FORMAT = "EZRC (arms.py): %s."

# /ezrassor/arm_back_velocity_controller/command
pub_FA = rospy.Publisher('/arm_front_velocity_controller/command', Float64, queue_size = 10)
pub_BA = rospy.Publisher('/arm_back_velocity_controller/command', Float64, queue_size = 10)


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

    # The arms are mirrored and the model so arm1 needs negative velocity to move up.

    # raise arm 1
    if arm1_up:
        pub_FA.publish(-arm_speed)

    # lower arm 1
    elif arm1_down:
        pub_FA.publish(arm_speed)

    else:
        pub_FA.publish(0)

    # raise arm 2
    if arm2_up:
        pub_BA.publish(arm_speed)

    # lower arm 2
    elif arm2_down:
        pub_BA.publish(-arm_speed)

    else:
        pub_BA.publish(0)


def start_node():
    # Main entry point to the node.
    try:
        print("Arms node started")
        rospy.init_node(NODE, anonymous=True)
        rospy.Subscriber(TOPIC, Int16, handle_arm_movements)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
