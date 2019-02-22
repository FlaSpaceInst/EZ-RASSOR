#!/usr/bin/env python
"""Control the flow of information through the EZ-RASSOR's ROS graph.

This node initially allows all user requests through to the movement toggles
topic until the user requests an AI routine, then that routine's responses are
allowed through and further user requests are ignored. This lasts until the
routine is finished, after which user requests are routed back to the movement
toggles topic.

Written by Tiger Sachse.
"""
import Queue
import rospy
import std_msgs


QUEUE_SIZE = 10
AI_KILL_MASK = 0b1000000000000
AI_TOGGLES_MASK = 0b1111000000000000
MOVEMENT_TOGGLES_MASK = 0b111111111111
NODE_NAME = "ezrassor_request_switcher"

REQUESTS_TOPIC = "/ezrassor/requests"
ROUTINE_TOGGLES_TOPIC = "/ezrassor/routine_toggles"
MOVEMENT_TOGGLES_TOPIC = "/ezrassor/movement_toggles"
ROUTINE_RESPONSES_TOPIC = "/ezrassor/routine_responses"


def get_masked_bits(bitstring, mask):
    """Get all masked bits from a bitstring."""
    result = bitstring & mask

    while mask % 2 == 0:
        result >>= 1
        mask >>= 1

    return result


def enqueue_bitstring(bitstring, additional_arguments):
    """Add a bitstring to a queue and associate it with its topic of origin.
    
    All additional arguments to this function are packaged as a tuple due to the
    *silly* nature of how ROS callbacks work.
    """
    queue, topic = additional_arguments
    queue.put((topic, bitstring), False)


# Main entry point to this node.
try:
    rospy.init_node(NODE_NAME)
    bitstring_queue = Queue.Queue()

    # Subscribe to the requests topic and routine responses topic.
    rospy.Subscriber(REQUESTS_TOPIC,
                     std_msgs.msg.Int16,
                     callback=enqueue_bitstring,
                     callback_args=(bitstring_queue, REQUESTS_TOPIC))
    rospy.Subscriber(ROUTINE_RESPONSES_TOPIC,
                     std_msgs.msg.Int16,
                     callback=enqueue_bitstring,
                     callback_args=(bitstring_queue, ROUTINE_RESPONSES_TOPIC))

    # Publish to the routine toggles topic and movement toggles topic.
    routine_toggles_publisher = rospy.Publisher(ROUTINE_TOGGLES_TOPIC,
                                                std_msgs.msg.Int8,
                                                queue_size=QUEUE_SIZE)
    movement_toggles_publisher = rospy.Publisher(MOVEMENT_TOGGLES_TOPIC,
                                                 std_msgs.msg.Int16,
                                                 queue_size=QUEUE_SIZE)
    
    ignoring_user = False
    while not rospy.core.is_shutdown():

        # If there is something new in the bitstring queue, dequeue it.
        try:
            topic, bitstring = bitstring_queue.get(False)
        except Queue.Empty:
            continue

        # Divide the bitstring into its components.
        kill_bit = get_masked_bits(bitstring.data, AI_KILL_MASK)
        ai_toggles = get_masked_bits(bitstring.data, AI_TOGGLES_MASK)
        movement_toggles = get_masked_bits(bitstring.data, MOVEMENT_TOGGLES_MASK)

        # Handle the bitstring according to the current state and the topic that
        # the bitstring came from.
        if ignoring_user:
            if topic == REQUESTS_TOPIC:
                continue
            elif kill_bit:
                ignoring_user = False
            else:
                movement_toggles_publisher.publish(movement_toggles)
        else:
            if topic == ROUTINE_RESPONSES_TOPIC:
                continue
            elif ai_toggles:
                ignoring_user = True
                routine_toggles_publisher.publish(ai_toggles)
            else:
                movement_toggles_publisher.publish(movement_toggles)
except KeyboardInterrupt:
    rospy.core.logdebug("keyboard interrupt, shutting down")
    rospy.core.signal_shutdown("keyboard_interrupt")
except rospy.ROSInterruptException:
    pass
