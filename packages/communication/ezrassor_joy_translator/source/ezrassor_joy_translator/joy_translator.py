"""Translate data from joy_node into something the EZ-RASSOR can understand.

Written by Harrison Black.
"""
import rospy
from std_msgs.msg import Float32, Int8
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

def callback(data, additional_arguments):
    """Parse Joy data, publish Twist data."""

    pub_wheels = additional_arguments[0]
    pub_front_arm = additional_arguments[1]
    pub_back_arm = additional_arguments[2]
    pub_front_drum = additional_arguments[3]
    pub_back_drum = additional_arguments[4]
    pub_auto_toggles = additional_arguments[5]

    # Raw controller input data indexes
    # data.buttons[index]
    # 0 A : Back Drum Dump
    # 1 B : Front Drum Dump
    # 2 X : Back Drum Dig
    # 3 Y : Front Drum Dig
    # 4 LB : Back Arm Up
    # 5 RB : Front Arm Up
    # 6 back : Tank Turn
    # 7 start : Supervisor Mode
    # 8 power : NA
    # 9 Button stick left : NA
    # 10 Button stick right : NA

    # data.axes[index]
    # 0 Left/Right Axis stick left : Turn Left/Right
    # 1 Up/Down Axis stick left : Move Forward/Reverse
    # 2 LT : Back Arm Down
    # 3 Left/Right Axis stick right : NA
    # 4 Up/Down Axis stick right : NA
    # 5 RT : Front Arm Down
    # 6 cross key left/right : Function 2/3
    # 7 cross key up/down : Function 1/4

    # AI Kill Bit
    if data.buttons[10] > 0:
        pub_auto_toggles.publish(0b100000)
        return
    # Auto Functions
    if data.axes[6] == 1:
        pub_auto_toggles.publish(0b000010)
        return
    elif data.axes[6] == -1:
        pub_auto_toggles.publish(0b000100)
        return
    elif data.axes[7] == 1:
        pub_auto_toggles.publish(0b000001)
        return
    elif data.axes[7] == -1:
        pub_auto_toggles.publish(0b001000)
        return
    elif data.buttons[9] > 0:
        pub_auto_toggles.publish(0b010000)
        return

    twist = Twist()
    twist.linear.x = ((data.axes[4] + data.axes[1]) / 2)
    twist.angular.z = ((data.axes[4] - data.axes[1]) / 2)
    trigger_threshold = 0.0

    # Use "-(1-data.axes[5])/2" not -1 for variable speed
    # Front Arm
    if data.buttons[5] > (1 - data.axes[5]) / 2:
    	command_front_arm = 1
    elif data.buttons[5] < (1 - data.axes[5]) / 2 and trigger_threshold > data.axes[5]:
    	command_front_arm = -1
    else:
    	command_front_arm = 0

    # Back Arm
    if data.buttons[4] > (1 - data.axes[2]) / 2:
    	command_back_arm = 1
    elif data.buttons[4] < (1 - data.axes[2]) / 2 and trigger_threshold > data.axes[2]:
    	command_back_arm = -1
    else:
    	command_back_arm = 0

    # Front Drum
    if data.buttons[3] > data.buttons[1]:
        command_front_drum = 1
    elif data.buttons[3] < data.buttons[1]:
    	command_front_drum = -1
    else:
    	command_front_drum = 0

    # Back Drum
    if data.buttons[2] > data.buttons[0]:
        command_back_drum = 1
    elif data.buttons[2] < data.buttons[0]:
    	command_back_drum = -1
    else:
    	command_back_drum = 0

    pub_wheels.publish(twist)
    pub_front_arm.publish(command_front_arm)
    pub_back_arm.publish(command_back_arm)
    pub_front_drum.publish(command_front_drum)
    pub_back_drum.publish(command_back_drum)
    

def start_node():
    try:
        TOPIC = "joy"
        NODE = "joy_translator"
        rospy.init_node(NODE)
        publish_topic_wheels = rospy.get_param(rospy.get_name()
                                               + "/wheel_instructions_topic")
        publish_topic_front_arm = rospy.get_param(rospy.get_name()
                                                  + "/front_arm_instructions_topic")
        publish_topic_back_arm = rospy.get_param(rospy.get_name()
                                                 + "/back_arm_instructions_topic")
        publish_topic_front_drum = rospy.get_param(rospy.get_name()
                                                   + "/front_drum_instructions_topic")
        publish_topic_back_drum = rospy.get_param(rospy.get_name()
                                                  + "/back_drum_instructions_topic")
        publish_topic_auto_toggles = "autonomous_toggles"

        # Publishers
        # Wheel twist
        pub_wheels = rospy.Publisher(publish_topic_wheels,
                                     Twist, 
                                     queue_size=10)
        # Arm Front
        pub_front_arm = rospy.Publisher(publish_topic_front_arm, 
                                        Float32, 
                                        queue_size=10)
        # Arm Back
        pub_back_arm = rospy.Publisher(publish_topic_back_arm, 
                                       Float32, 
                                       queue_size=10)
        # Drum Front
        pub_front_drum = rospy.Publisher(publish_topic_front_drum, 
                                         Float32, 
                                         queue_size=10)
        # Drum Back
        pub_back_drum = rospy.Publisher(publish_topic_back_drum,
                                        Float32, 
                                        queue_size=10)
        # Autonomous Toggles
        pub_auto_toggles = rospy.Publisher(publish_topic_auto_toggles,
                                           Int8,
                                           queue_size=10)
        rate = rospy.Rate(60)
        rospy.Subscriber(TOPIC,
                         Joy,
                         callback,
                         callback_args=(pub_wheels,
                                        pub_front_arm,
                                        pub_back_arm,
                                        pub_front_drum,
                                        pub_back_drum,
                                        pub_auto_toggles))
        rospy.loginfo("Joy translator initialized.")
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
