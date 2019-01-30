#!/usr/bin/env python
import rospy
from std_msgs.msg import Int8, Int16


pub = rospy.Publisher('ezmain_topic', Int16, queue_size=100)
rospy.init_node('ai_control_node', anonymous=True)

rate = rospy.Rate(1) # 1hz

def ai_control():
    while not rospy.is_shutdown():
        pub.publish(0b1000)
        rate.sleep()


if __name__ == "__main__":
    try:
        ai_control()
    except rospy.ROSInterruptException:
        pass