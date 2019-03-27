#!/usr/bin/env python
"""A ROS node that moves the wheel on the EZRC.

Written by Harrison Black and Tiger Sachse.
Part of the EZ-RASSOR suite of software.
"""
import rospy
from std_msgs.msg import Int16, Float64

NODE = "wheels"
TOPIC = "/ezrassor/movement_toggles"
MASK = 0b111100000000
velocity = 5
# /ez_rassor/left_wheel_back_velocity_controller/command
pub_LF = rospy.Publisher('/ez_rassor/left_wheel_front_velocity_controller/command', Float64, queue_size = 10)
pub_LB = rospy.Publisher('/ez_rassor/left_wheel_back_velocity_controller/command', Float64, queue_size = 10)
pub_RF = rospy.Publisher('/ez_rassor/right_wheel_front_velocity_controller/command', Float64, queue_size = 10)
pub_RB = rospy.Publisher('/ez_rassor/right_wheel_back_velocity_controller/command', Float64, queue_size = 10)


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


def wheel_movement_callback(instruction):
    left_wheel_forward, left_wheel_reverse, right_wheel_forward, right_wheel_reverse = get_movements(instruction.data, MASK)


    # Turning behaves odd with offset and tankturn. Set to zero for now.
    turn_offset = 0

    if left_wheel_forward:
        pub_LF.publish(velocity)
        pub_LB.publish(velocity)

    elif left_wheel_reverse:
        pub_LF.publish(-velocity + turn_offset)
        pub_LB.publish(-velocity + turn_offset)

    else:
        pub_LF.publish(0)
        pub_LB.publish(0)

    if right_wheel_forward:
        pub_RF.publish(velocity)
        pub_RB.publish(velocity)

    elif right_wheel_reverse:
        pub_RF.publish(-velocity + turn_offset)
        pub_RB.publish(-velocity + turn_offset)

    else:
        pub_RF.publish(0)
        pub_RB.publish(0)


    # print(data_string)

def main():

    print("Wheel node started")
    rospy.init_node(NODE, anonymous = True)
    rospy.Subscriber(TOPIC, Int16, wheel_movement_callback)
    rospy.spin()

if __name__ == '__main__':

    try:
        main()

    except rospy.ROSInterruptException:
        pass
