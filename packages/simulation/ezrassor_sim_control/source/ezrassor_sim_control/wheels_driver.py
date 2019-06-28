"""A ROS node that moves the wheel on the simulated robot.

Written by Harrison Black.
"""
import rospy
from std_msgs.msg import Int16, Float64
from geometry_msgs.msg import Twist

NODE = "wheels"
TOPIC = "wheel_instructions"
MAX_VELOCITY = 5

pub_LF = rospy.Publisher('left_wheel_front_velocity_controller/command', Float64, queue_size = 10)
pub_LB = rospy.Publisher('left_wheel_back_velocity_controller/command', Float64, queue_size = 10)
pub_RF = rospy.Publisher('right_wheel_front_velocity_controller/command', Float64, queue_size = 10)
pub_RB = rospy.Publisher('right_wheel_back_velocity_controller/command', Float64, queue_size = 10)

def wheel_movement_callback(twist):
    x = twist.linear.x
    z = twist.angular.z

    pub_LB.publish((x-z)*MAX_VELOCITY)
    pub_LF.publish((x-z)*MAX_VELOCITY)
    pub_RB.publish((x+z)*MAX_VELOCITY)
    pub_RF.publish((x+z)*MAX_VELOCITY)


def start_node():
    try:
        rospy.init_node(NODE, anonymous = True)
        rospy.Subscriber(TOPIC, Twist, wheel_movement_callback)
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
