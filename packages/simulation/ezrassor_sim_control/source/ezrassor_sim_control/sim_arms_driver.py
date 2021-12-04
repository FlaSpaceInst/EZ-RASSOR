"""A ROS node that moves the arms of the simulated robot.

Written by Harrison Black.
"""
import rospy
from std_msgs.msg import Float32, Float64

NODE = "sim_arms_driver"
FRONT_TOPIC = "front_arm_instructions"
BACK_TOPIC = "back_arm_instructions"
MAX_ARM_SPEED = 0.75

# /ezrassor/arm_back_velocity_controller/command
pub_FA = rospy.Publisher(
    "arm_front_velocity_controller/command", Float64, queue_size=10
)
pub_BA = rospy.Publisher(
    "arm_back_velocity_controller/command", Float64, queue_size=10
)


def handle_front_arm_movements(data):
    """Move the front arm of the robot per
    the commands encoded in the instruction.
    """
    pub_FA.publish(data.data * MAX_ARM_SPEED)


def handle_back_arm_movements(data):
    """Move the back arm of the robot per
    the commands encoded in the instruction.
    """
    pub_BA.publish(data.data * MAX_ARM_SPEED)


def start_node(rover_model):
    # Main entry point to the node.
    try:
        rospy.init_node(NODE)
        if rover_model != "paver_arm":
            rospy.Subscriber(FRONT_TOPIC, Float32, handle_front_arm_movements)
        rospy.Subscriber(BACK_TOPIC, Float32, handle_back_arm_movements)
        rospy.loginfo("Simulation arms driver initialized.")
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
