"""Control the flow of information through the EZ-RASSOR's ROS graph.

Written by Tiger Sachse.
"""
import rospy
import std_msgs
import geometry_msgs


class OverrideStatus:
    """"""
    def __init__(self, override_topic):
        self.status = False
        rospy.Subscriber(
            override_topic,
            std_msgs.msg.Bool,
            callback=self.update,
        )
    
    def __eq__(self, other_status):
        """"""
        return self.status == other_status

    def update(self, data):
        """"""
        self.status = data.data


def route_item(data, additional_arguments):
    """"""
    output_publisher, override_status, expected_status = additional_arguments

    if override_status == expected_status:
        output_publisher.publish(data.data)


def get_topic_class(topic_type):
    """"""
    if topic_type == "Float32":
        return std_msgs.msg.Float32
    elif topic_type == "Twist":
        return geometry_msgs.msg.Twist
    else:
        raise TypeError("Unsupported topic type '%s'" % topic_type)


def start_node(
    node_name,
    primary_topic,
    secondary_topic,
    output_topic,
    override_topic,
    topic_type,
    queue_size):
    """Fire up this node!"""
    try:
        rospy.init_node(node_name, anonymous=True)
        override_status = OverrideStatus(override_topic)
        topic_class = get_topic_class(topic_type)

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

        rospy.spin()
    except rospy.ROSInterruptException:
        pass
