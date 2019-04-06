"""A ROS node that moves the wheel on the EZRC.

Written by Harrison Black and Tiger Sachse.
Part of the EZ-RASSOR suite of software.
"""
import rospy
from std_msgs.msg import Int16, Float64
from geometry_msgs.msg import Twist

NODE = "wheels"
TOPIC = "/wheel_instructions"
MAX_VELOCITY = 3

# /ezrassor/left_wheel_back_velocity_controller/command
pub_LF = rospy.Publisher('/left_wheel_front_velocity_controller/command', Float64, queue_size = 10)
pub_LB = rospy.Publisher('/left_wheel_back_velocity_controller/command', Float64, queue_size = 10)
pub_RF = rospy.Publisher('/right_wheel_front_velocity_controller/command', Float64, queue_size = 10)
pub_RB = rospy.Publisher('/right_wheel_back_velocity_controller/command', Float64, queue_size = 10)

def wheel_movement_callback(twist):
    x = twist.linear.x
    z = twist.angular.z
    
    if x != 0:
        pub_LB.publish(x)
        pub_LF.publish(x)
        pub_RB.publish(x)
        pub_RF.publish(x)
    elif twist.angular.z != 0:
        pub_LB.publish(-z)
        pub_LF.publish(-z)
        pub_RB.publish(z)
        pub_RF.publish(z)
    else:
        pub_LB.publish(0)
        pub_LF.publish(0)
        pub_RB.publish(0)
        pub_RF.publish(0)

def start_node():
    try:
        rospy.init_node(NODE, anonymous = True)
        rospy.Subscriber(TOPIC, Twist, wheel_movement_callback)
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
