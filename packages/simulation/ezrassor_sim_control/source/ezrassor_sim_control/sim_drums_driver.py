"""A ROS node that moves the drums on the simulated robot.

Written by Harrison Black.
"""
import rospy
import std_msgs
from std_msgs.msg import Float32, Float64

NODE = "sim_drums_driver"
FRONT_TOPIC = "front_drum_instructions"
BACK_TOPIC = "back_drum_instructions"
MAX_DRUM_SPEED = 5

# /ezrassor/drum_back_velocity_controller/command
pub_FA = rospy.Publisher('drum_front_velocity_controller/command', 
                         Float64, 
                         queue_size = 10)
pub_BA = rospy.Publisher('drum_back_velocity_controller/command', 
                         Float64, 
                         queue_size = 10)

def handle_front_drum_movements(data):
    """Move the front drum of the robot per 
        the commands encoded in the instruction.
    """
    pub_FA.publish(data.data*MAX_DRUM_SPEED)

def handle_back_drum_movements(data):
    """Move the back drum of the robot per 
        the commands encoded in the instruction.
    """
    pub_BA.publish(data.data*MAX_DRUM_SPEED)


def start_node():
    # Main entry point to the node.
    try:
        rospy.init_node(NODE)
        rospy.Subscriber(FRONT_TOPIC, Float32, handle_front_drum_movements)
        rospy.Subscriber(BACK_TOPIC, Float32, handle_back_drum_movements)
        rospy.loginfo("Simulation drums driver initialized.")
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

