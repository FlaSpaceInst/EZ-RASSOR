#!/usr/bin/env python
import rospy
import std_msgs
import Queue

AI_TOPIC = "in1"
USER_TOPIC = "in2"

def process_this(thing, args):
    print(args)
    args[0].put((args[1], thing), False)

try:
    rospy.init_node("controller", anonymous=True)

    q = Queue.Queue()
    rospy.Subscriber(AI_TOPIC, std_msgs.msg.Int16, callback=process_this, callback_args=(q, AI_TOPIC))
    rospy.Subscriber(USER_TOPIC, std_msgs.msg.Int16, callback=process_this, callback_args=(q, USER_TOPIC))
    pub = rospy.Publisher("out", std_msgs.msg.Int16, queue_size=10)
    cry = rospy.Publisher("cry", std_msgs.msg.Int16, queue_size=10)
    try:
        ai_only = False
        while not rospy.core.is_shutdown():
            try:
                topic, thing = q.get(False)
            except Queue.Empty:
                continue

            if thing.data > 5000:
                ai_only = True
                cry.publish(9000)
            elif thing.data == -1:
                ai_only = False

            if ai_only == True and topic == USER_TOPIC:
                continue
            else:
                pub.publish(thing)
    except KeyboardInterrupt:
        # log
        rospy.core.signal_shutdown('keyboard_interrupt')
except rospy.ROSInterruptException:
    pass
