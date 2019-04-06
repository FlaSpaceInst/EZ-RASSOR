"""A ROS node that moves the arms on the EZRC.

Written by Harrison Black and Tiger Sachse.
Part of the EZ-RASSOR suite of software.
"""
import rospy
import std_msgs
from std_msgs.msg import Float32, Float64

NODE = "arms"
FRONT_TOPIC = "/front_arm_instructions"
BACK_TOPIC = "/back_arm_instructions"
MAX_ARM_SPEED = 1

# /ezrassor/arm_back_velocity_controller/command
pub_FA = rospy.Publisher('/arm_front_velocity_controller/command', 
                         Float64, 
                         queue_size = 10)
pub_BA = rospy.Publisher('/arm_back_velocity_controller/command', 
                         Float64, 
                         queue_size = 10)

def handle_front_arm_movements(data):
    """Move the front arm of the EZRC per 
        the commands encoded in the instruction.
    """
    pub_FA.publish(data.data*MAX_ARM_SPEED)

def handle_back_arm_movements(data):
    """Move the back arm of the EZRC per 
        the commands encoded in the instruction.
    """
    pub_BA.publish(data.data*MAX_ARM_SPEED)


def start_node():
    # Main entry point to the node.
    try:
        rospy.init_node(NODE, anonymous=True)
        rospy.Subscriber(FRONT_TOPIC, Float32, handle_front_arm_movements)
        rospy.Subscriber(BACK_TOPIC, Float32, handle_back_arm_movements)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
