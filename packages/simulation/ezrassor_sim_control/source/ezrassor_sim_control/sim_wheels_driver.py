"""A ROS node that moves the wheel on the simulated robot.

Written by Harrison Black.
"""
import rospy
from std_msgs.msg import Int16, Float64
from geometry_msgs.msg import Twist

NODE = "sim_wheels_driver"
TOPIC = "wheel_instructions"
MAX_VELOCITY = 5

#pub_LF = rospy.Publisher('left_wheel_front_velocity_controller/command', Float64, queue_size = 10)
#pub_LB = rospy.Publisher('left_wheel_back_velocity_controller/command', Float64, queue_size = 10)
#pub_RF = rospy.Publisher('right_wheel_front_velocity_controller/command', Float64, queue_size = 10)
#pub_RB = rospy.Publisher('right_wheel_back_velocity_controller/command', Float64, queue_size = 10)

def wheel_movement_callback(twist):
    #pub_front_wheels = rospy.Publisher('diff_drive_controller_front/cmd_vel', Twist, queue_size=10)
    pub_wheels = rospy.Publisher('diff_drive_controller/cmd_vel', Twist, queue_size=10)
    new_twist = Twist()
    new_twist.linear.x = twist.linear.x * MAX_VELOCITY
    new_twist.linear.y = twist.linear.y
    new_twist.linear.z = twist.linear.z
    new_twist.angular.x = twist.angular.x
    new_twist.angular.y = twist.angular.y
    new_twist.angular.z = twist.angular.z * MAX_VELOCITY
    #pub_back_wheels.publish(new_twist)
    pub_wheels.publish(new_twist)

    #pub_LB.publish((x-z)*MAX_VELOCITY)
    #pub_LF.publish((x-z)*MAX_VELOCITY)
    #pub_RB.publish((x+z)*MAX_VELOCITY)
    #pub_RF.publish((x+z)*MAX_VELOCITY)


def start_node():
    try:
        rospy.init_node(NODE, anonymous = True)
        rospy.Subscriber(TOPIC, Twist, wheel_movement_callback)
        rospy.loginfo("Simulation wheels driver initialized.")
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
