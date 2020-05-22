"""A ROS node that moves the wheel on the simulated robot.

Written by Harrison Black and Shelby Basco.
"""
import rospy
from std_msgs.msg import Int16, Float64
from geometry_msgs.msg import Twist

NODE = "sim_wheels_driver"
TOPIC = "wheel_instructions"
MAX_VELOCITY = 5

def wheel_movement_callback(twist):

    pub_wheels = rospy.Publisher('diff_drive_controller/cmd_vel', Twist, queue_size=10)

    new_twist = Twist()

    # The factor of MAX_VELOCITY was included in EZ-RASSOR 1.0 as well
    # Although there are caps on the speed in the diff_drive, this helps the
    # rover not turn incredibly slow when making lefts, rights, donuts, etc.

    new_twist.linear.x = twist.linear.x * MAX_VELOCITY
    new_twist.linear.y = twist.linear.y
    new_twist.linear.z = twist.linear.z
    new_twist.angular.x = twist.angular.x
    new_twist.angular.y = twist.angular.y
    new_twist.angular.z = twist.angular.z * MAX_VELOCITY

    pub_wheels.publish(new_twist)


def start_node():
    try:
        rospy.init_node(NODE, anonymous = True)
        rospy.Subscriber(TOPIC, Twist, wheel_movement_callback)
        rospy.loginfo("Simulation wheels driver initialized.")
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
