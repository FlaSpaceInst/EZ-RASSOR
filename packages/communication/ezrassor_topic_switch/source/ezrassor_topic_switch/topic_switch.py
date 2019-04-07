"""Control the flow of information through the EZ-RASSOR's ROS graph.

Written by Tiger Sachse.
"""
import rospy
import std_msgs.msg
import geometry_msgs.msg


class OverrideStatus:
    """Automatically maintain an override status."""
    def __init__(self, override_topic):
        """Initialize this override status with a subscriber."""
        self.status = False
        rospy.Subscriber(
            override_topic,
            std_msgs.msg.Bool,
            callback=self.update,
        )
    
    def __eq__(self, other_status):
        """Allow easy comparisons of this override status with a boolean."""
        return self.status == other_status

    def update(self, data):
        """Update the override status with data from the subscriber."""
        self.status = data.data


def route_item(data, additional_arguments):
    """Route an item to the output topic if the override status permits it."""
    output_publisher, override_status, expected_status = additional_arguments

    if override_status == expected_status:
        output_publisher.publish(data.data)


def get_topic_class(topic_type):
    """Get a topic class from a given topic type.
    
    This is a horrible way to do this. :) Please generalize if you have the time.
    """
    if topic_type == "Float32":
        return std_msgs.msg.Float32
    elif topic_type == "Twist":
        return geometry_msgs.msg.Twist
    else:
        raise TypeError("Unsupported topic type '%s'" % topic_type)


def start_node(default_node_name, override_topic, queue_size):
    """Fire up this node!"""
    try:
        rospy.init_node(default_node_name)

        # Load in parameters from the parameter server.
        parameter_prefix = rospy.get_name()
        primary_topic = rospy.get_param(parameter_prefix + "/primary_topic")
        secondary_topic = rospy.get_param(parameter_prefix + "/secondary_topic")
        output_topic = rospy.get_param(parameter_prefix + "/output_topic")
        topic_type = rospy.get_param(parameter_prefix + "/topic_type")

        override_status = OverrideStatus(override_topic)
        topic_class = get_topic_class(topic_type)

        # Create all publishers and subscribers.
        output_publisher = rospy.Publisher(
            output_topic,
            topic_class,
            queue_size=queue_size,
        )
        rospy.Subscriber(
            primary_topic,
            topic_class,
            callback=route_item,
            callback_args=(
                output_publisher,
                override_status,
                False,
            ),
        )
        rospy.Subscriber(
            secondary_topic,
            topic_class,
            callback=route_item,
            callback_args=(
                output_publisher,
                override_status,
                True,
            ),
        )

        # Keep this baby alive!
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
