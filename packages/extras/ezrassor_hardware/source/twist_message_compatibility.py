#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16

twist_pub = rospy.Publisher('/ezrassor/hardware_wheels', Twist, queue_size=100)
LINEAR_VELOCITY = 5
ANGULAR_VELOCITY = .5

def twist_callback(data):
    twist_message = Twist()
    data = data.data
    print(data)
    if data==2560:
        print("Forward")
        twist_message.linear.x = LINEAR_VELOCITY
    elif data==1280:
        print("Reverse")
        twist_message.linear.x = -LINEAR_VELOCITY
    elif data==2304:
        twist_message.angular.z = - ANGULAR_VELOCITY
    elif data==1536:
        twist_message.angular.z = ANGULAR_VELOCITY
    else:
        pass
    
    twist_pub.publish(twist_message)


def run():
    rospy.init_node("HardwareCompat")

    rospy.Subscriber('/ezrassor/movement_toggles', Int16, twist_callback)
    rospy.Publisher('/ezrassor/hardware_wheels', Twist, queue_size=100)

    rospy.spin()



if __name__ == "__main__":
    run()