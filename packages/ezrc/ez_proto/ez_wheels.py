#!/usr/bin/env python
import rospy
from std_msgs.msg import Int16, Float64

pubLF = rospy.Publisher('/ez_rassor/left_wheel_front_velocity_controller/command', Float64, queue_size = 10)
pubLB = rospy.Publisher('/ez_rassor/left_wheel_back_velocity_controller/command', Float64, queue_size = 10)
pubRF = rospy.Publisher('/ez_rassor/right_wheel_front_velocity_controller/command', Float64, queue_size = 10)
pubRB = rospy.Publisher('/ez_rassor/right_wheel_back_velocity_controller/command', Float64, queue_size = 10)


def callback(data):

    data_in = data.data
    
    mask = 0b111100000000
    data_in &= mask
    data_in >>= 8
    data_string = "Wheels:\t {0:04b}".format(data_in)

    velocity = 10

    if data_in == 0b1000:
        data_string = data_string + " -> Drive Forward"
        pubLF.publish(velocity)
        pubLB.publish(velocity)
        pubRF.publish(velocity)
        pubRB.publish(velocity)

    elif data_in == 0b0100:
        data_string = data_string + " -> Reverse"

        # Insert motor functions

    elif data_in == 0b0010:
        data_string = data_string + " -> Turn Left"

        # Insert motor functions

    elif data_in == 0b0001:
        data_string = data_string + " -> Turn Right"

        # Insert motor functions

    else:
        data_string = data_string + " -> Stop"

        # Halt motor functions

    print(data_string)

def main():

    rospy.init_node('ez_arms', anonymous = True)
    rospy.Subscriber('ezmain_topic', Int16, callback)
    rospy.spin()

if __name__ == '__main__':

    try:
        main()

    except rospy.ROSInterruptException:
        pass