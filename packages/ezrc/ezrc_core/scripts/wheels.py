#!/usr/bin/env python
import rospy
from std_msgs.msg import Int16, Float64

MASK = 0b111100000000

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
    # print("callback")
    # data_in = data.data

    # # mask = 0b111100000000
    # data_in &= mask
    # data_in >>= 8
    # data_string = "Wheels:\t {0:04b}".format(data_in)

    drive_forward, drive_reverse, turn_left, turn_right = get_movements(instruction.data, MASK)

    velocity = 5
    turn_offset = 0.05

    if drive_forward:
        # data_string = data_string + " -> Drive Forward"
        pub_LF.publish(velocity)
        pub_LB.publish(velocity)
        pub_RF.publish(velocity)
        pub_RB.publish(velocity)

    elif drive_reverse:
        # data_string = data_string + " -> Reverse"
        pub_LF.publish(-velocity)
        pub_LB.publish(-velocity)
        pub_RF.publish(-velocity)
        pub_RB.publish(-velocity)

    elif turn_left:
        # data_string = data_string + " -> Turn Left"
        pub_LF.publish(-velocity + turn_offset)
        pub_LB.publish(-velocity + turn_offset)
        pub_RF.publish(velocity)
        pub_RB.publish(velocity)

    elif turn_right:
        # data_string = data_string + " -> Turn Right"
        pub_LF.publish(velocity)
        pub_LB.publish(velocity)
        pub_RF.publish(-velocity + turn_offset)
        pub_RB.publish(-velocity + turn_offset)

    else:
        # data_string = data_string + " -> Stop"
        # Halt motor functions
        pub_LF.publish(0)
        pub_LB.publish(0)
        pub_RF.publish(0)
        pub_RB.publish(0)



    # print(data_string)

def main():

    print("Wheel node started")
    rospy.init_node('ez_wheels', anonymous = True)
    rospy.Subscriber('ez_main_topic', Int16, wheel_movement_callback)
    rospy.spin()

if __name__ == '__main__':

    try:
        main()

    except rospy.ROSInterruptException:
        pass