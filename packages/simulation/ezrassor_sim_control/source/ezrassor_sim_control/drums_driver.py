"""A ROS node that moves the drums on the EZRC.

Written by Harrison Black and Tiger Sachse.
Part of the EZ-RASSOR suite of software.
"""
import rospy
import std_msgs
from std_msgs.msg import Float32, Float64

NODE = "drums"
FRONT_TOPIC = "front_drum_instruction"
BACK_TOPIC = "back_drum_instruction"
MAX_DRUM_SPEED = 1

# /ezrassor/drum_back_velocity_controller/command
pub_FA = rospy.Publisher('drum_front_velocity_controller/command', 
                         Float64, 
                         queue_size = 10)
pub_BA = rospy.Publisher('drum_back_velocity_controller/command', 
                         Float64, 
                         queue_size = 10)

def handle_front_drum_movements(data):
    """Move the front drum of the EZRC per 
        the commands encoded in the instruction.
    """
    pub_FA.publish(data.data*MAX_DRUM_SPEED)

def handle_back_drum_movements(data):
    """Move the back drum of the EZRC per 
        the commands encoded in the instruction.
    """
    pub_BA.publish(data.data*MAX_DRUM_SPEED)


def start_node():
    # Main entry point to the node.
    try:
        rospy.init_node(NODE, anonymous=True)
        rospy.Subscriber(FRONT_TOPIC, Float32, handle_front_drum_movements)
        rospy.Subscriber(BACK_TOPIC, Float32, handle_back_drum_movements)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

