"""Control the flow of information through a ROS graph.

Written by Tiger Sachse.
"""
import rospy
import threading
import std_msgs.msg


class OverrideStatus:
    """Automatically maintain an override status."""
    def __init__(self, override_topic):
        """Initialize this override status with a subscriber.
        
        This class possibly doesn't need the lock, but I left it just in case.
        """
        self.__status = False
        rospy.Subscriber(
            override_topic,
            std_msgs.msg.Bool,
            callback=self.__update,
        )
        self.__status_lock = threading.Lock()
    
    def __eq__(self, other_status):
        """Allow easy comparisons of this override status with a boolean."""
        self.__status_lock.acquire()
        status_equality = self.__status == other_status
        self.__status_lock.release()

        return status_equality

    def __update(self, status):
        """Update the override status with data from the subscriber."""
        self.__status_lock.acquire()
        self.__status = status.data
        self.__status_lock.release()


def route_item(item, additional_arguments):
    """Route an item to the output topic if the override status permits it."""
    output_publisher, override_status, expected_status = additional_arguments

    if override_status == expected_status:
        output_publisher.publish(item)


def start_node(default_node_name, override_topic, queue_size):
    """Fire up the switch."""
    try:
        rospy.init_node(default_node_name)

        # Load in parameters from the parameter server.
        primary_topic = rospy.get_param(rospy.get_name() + "/primary_topic")
        secondary_topic = rospy.get_param(rospy.get_name() + "/secondary_topic")
        output_topic = rospy.get_param(rospy.get_name() + "/output_topic")
        topic_type_module = rospy.get_param(rospy.get_name() + "/topic_type_module")
        topic_type_class = rospy.get_param(rospy.get_name() + "/topic_type_class")
        override_status = OverrideStatus(override_topic)

        # Overwrite the topic type module and topic type class with an imported
        # module and class. If these can't be imported, an exception is thrown.
        try:
            topic_type_module = __import__(
                topic_type_module,
                globals(),
                locals(),
                [topic_type_class],
                -1,
            )
            topic_type_class = vars(topic_type_module)[topic_type_class]
        except ImportError:
            raise ImportError(
                "No topic type module named \"{0}\"".format(str(topic_type_module)),
            )
        except KeyError:
            raise ImportError(
                "No topic type class named \"{0}\"".format(str(topic_type_class)),
            )

        # Create all publishers and subscribers.
        output_publisher = rospy.Publisher(
            output_topic,
            topic_type_class,
            queue_size=queue_size,
        )
        rospy.Subscriber(
            primary_topic,
            topic_type_class,
            callback=route_item,
            callback_args=(
                output_publisher,
                override_status,
                False,
            ),
        )
        rospy.Subscriber(
            secondary_topic,
            topic_type_class,
            callback=route_item,
            callback_args=(
                output_publisher,
                override_status,
                True,
            ),
        )

        rospy.loginfo("Topic switch initialized.")

        # Keep this baby alive!
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
