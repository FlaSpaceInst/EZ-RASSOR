#!/usr/bin/env python
import rospy
from std_msgs.msg import Int16, Float64

NODE = "drums"
TOPIC = "ez_main_topic"
MASK = 0b000000001111


# /ez_rassor/drum_back_velocity_controller/command
# /ez_rassor/drum_front_velocity_controller/command
pub_FD = rospy.Publisher('/ez_rassor/drum_front_velocity_controller/command', Float64, queue_size = 10)
pub_BD = rospy.Publisher('/ez_rassor/drum_back_velocity_controller/command', Float64, queue_size = 10)



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

def drum_movement_callback(instruction):

    front_dig, front_dump, back_dig, back_dump = get_movements(instruction.data, MASK)
    drum_speed = 5

    if not any((front_dig, front_dump, back_dig, back_dump)):
        # stop both drums
        pub_FD.publish(0)
        pub_BD.publish(0)

    else:
        if front_dig:
            pub_FD.publish(drum_speed)

        if front_dump:
            pub_FD.publish(-drum_speed)

        if back_dig:
            pub_BD.publish(-drum_speed)

        if back_dump:
            pub_BD.publish(drum_speed)


def main():
    print("Drums node started")
    rospy.init_node('ez_arms', anonymous = True)
    rospy.Subscriber('ez_main_topic', Int16, drum_movement_callback)
    rospy.spin()

if __name__ == '__main__':

    try:
        main()

    except rospy.ROSInterruptException:
        pass