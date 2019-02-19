#!/usr/bin/env python
""""""
import Queue
import rospy
import std_msgs


QUEUE_SIZE = 10
AI_KILL_MASK = 0b1000000000000
AI_COMMANDS_MASK = 0b1111000000000000
MOVEMENT_COMMANDS_MASK = 0b111111111111

NODE_NAME = "ezrassor_switcher"
AI_TRIGGER_TOPIC = "ai_triggers"
AI_COMMANDS_TOPIC = "ai_commands"
USER_COMMANDS_TOPIC = "user_commands"
MOVEMENT_COMMANDS_TOPIC = "movement_commands"


def get_masked_bits(bitstring, mask):
    """"""
    result = bitstring & mask

    while mask % 2 == 0:
        result >>= 1
        mask >>= 1

    return result

def enqueue_command(command, additional_arguments):
    """"""
    queue, topic = additional_arguments
    queue.put((topic, command), False)

try:
    rospy.init_node(NODE_NAME)
    commands_queue = Queue.Queue()

    rospy.Subscriber(AI_COMMANDS_TOPIC,
                     std_msgs.msg.Int16,
                     callback=enqueue_command,
                     callback_args=(commands_queue, AI_COMMANDS_TOPIC))
    rospy.Subscriber(USER_COMMANDS_TOPIC,
                     std_msgs.msg.Int16,
                     callback=enqueue_command,
                     callback_args=(commands_queue, USER_COMMANDS_TOPIC))
    trigger_publisher = rospy.Publisher(AI_TRIGGER_TOPIC,
                                        std_msgs.msg.Int16,
                                        queue_size=QUEUE_SIZE)
    movement_publisher = rospy.Publisher(MOVEMENT_COMMANDS_TOPIC,
                                         std_msgs.msg.Int16,
                                         queue_size=QUEUE_SIZE)
    
    ignoring_user = False 
    while not rospy.core.is_shutdown():
        try:
            topic, command = commands_queue.get(False)
        except Queue.Empty:
            continue

        kill_bit = get_masked_bits(command.data, AI_KILL_MASK)
        ai_command_bits = get_masked_bits(command.data, AI_COMMANDS_MASK)
        movement_bits = get_masked_bits(command.data, MOVEMENT_COMMANDS_MASK)

        if ignoring_user:
            if topic == USER_COMMANDS_TOPIC:
                continue
            elif kill_bit:
                ignoring_user = False
                continue
            else:
                movement_publisher.publish(movement_bits)
        else:
            if topic == AI_COMMANDS_TOPIC:
                continue
            elif ai_command_bits != 0:
                trigger_publisher.publish(ai_command_bits)
                ignoring_user = True
            else:
                movement_publisher.publish(movement_bits)

except rospy.ROSInterruptException:
    # missing logging line?
    rospy.core.signal_shutdown("keyboard_interrupt")
