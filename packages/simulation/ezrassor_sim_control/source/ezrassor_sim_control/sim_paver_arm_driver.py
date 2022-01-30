#!/user/bin/env python

import rospy
from std_msgs.msg import (
    Float32,
    Float64,
    Float64MultiArray,
    MultiArrayDimension,
)

NODE = "sim_paver_arm_driver"
FIRST_JOINT_TOPIC = "paver_arm_joint_1_instructions"
SECOND_JOINT_TOPIC = "paver_arm_joint_2_instructions"
THIRD_JOINT_TOPIC = "paver_arm_joint_3_instructions"
FOURTH_JOINT_TOPIC = "paver_arm_joint_4_instructions"
FIFTH_JOINT_TOPIC = "paver_arm_joint_5_instructions"
CLAW_TOPIC = "paver_arm_claw_instructions"
AUTO_TOPIC = "paver_arm_controller_instructions"

pub_1J = rospy.Publisher(
    "PA_first_joint_position_controller/command", Float64, queue_size=10
)

pub_2J = rospy.Publisher(
    "PA_second_joint_position_controller/command", Float64, queue_size=10
)

pub_3J = rospy.Publisher(
    "PA_third_joint_position_controller/command", Float64, queue_size=10
)

pub_4J = rospy.Publisher(
    "PA_fourth_joint_position_controller/command", Float64, queue_size=10
)

pub_5J = rospy.Publisher(
    "PA_fifth_joint_position_controller/command", Float64, queue_size=10
)

pub_claw = rospy.Publisher(
    "PA_claw_velocity_controller/command", Float64MultiArray, queue_size=10
)

# pub_auto = rospy.Publisher(
#    "paver_arm_controller/command", JointTrajectoryPoint, queue_size=10
# )


def handle_first_joint_movements(data):

    pub_1J.publish(data.data)


def handle_second_joint_movements(data):

    pub_2J.publish(data.data)


def handle_third_joint_movements(data):

    pub_3J.publish(data.data)


def handle_fourth_joint_movements(data):

    pub_4J.publish(data.data)


def handle_fifth_joint_movements(data):

    pub_5J.publish(data.data)


def handle_claw_movements(data):
    msg = Float64MultiArray()
    msg.layout.dim.append(MultiArrayDimension())
    msg.layout.dim[0].size = 2
    msg.layout.dim[0].stride = 1
    msg.layout.dim[0].label = "x"
    msg.data = [data.data] * 2
    pub_claw.publish(msg)


# def  handle_auto_movement(data):

#   pub_auto.publish(data.data)


def start_node():
    try:
        rospy.init_node(NODE)
        rospy.Subscriber(
            FIRST_JOINT_TOPIC, Float64, handle_first_joint_movements
        )
        rospy.Subscriber(
            SECOND_JOINT_TOPIC, Float64, handle_second_joint_movements
        )
        rospy.Subscriber(
            THIRD_JOINT_TOPIC, Float64, handle_third_joint_movements
        )
        rospy.Subscriber(
            FOURTH_JOINT_TOPIC, Float64, handle_fourth_joint_movements
        )
        rospy.Subscriber(
            FIFTH_JOINT_TOPIC, Float64, handle_fifth_joint_movements
        )
        rospy.Subscriber(CLAW_TOPIC, Float32, handle_claw_movements)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
