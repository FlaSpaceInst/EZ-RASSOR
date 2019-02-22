#!/usr/bin/env python
"""Translate API requests into ROS bitstring messages.

Written by Camilo Lozano.
Edited by Tiger Sachse.
"""
import json
import rospy
import flask
import Queue
import std_msgs
import multiprocessing


QUEUE_SIZE = 10
HOST_ADDRESS = "0.0.0.0"
NODE_NAME = "restful_api"
COMMAND_URL = "/post_command"
REQUESTS_TOPIC = "/ezrassor/requests"


try:

    
    request_queue = multiprocessing.Queue()

    def handle_posted_command():
        """"""
        q.put(100, False)
        #requests_publisher.publish(int(flask.request.data))

        return (json.dumps({COMMAND_URL : int(flask.request.data)}), 200)

    rospy.init_node(NODE_NAME)
    requests_publisher = rospy.Publisher(REQUESTS_TOPIC,
                                         std_msgs.msg.Int16,
                                         queue_size=QUEUE_SIZE)
    app = flask.Flask(__name__)
    app.add_url_rule(COMMAND_URL, view_func=handle_posted_command, methods=["POST"])
    server = multiprocessing.Process(target=app.run, args=(HOST_ADDRESS,))
    server.start()

    while not rospy.core.is_shutdown():
        try:
            number = q.get(False)
            requests_publisher.publish(number)
        except Queue.Empty:
            continue
except rospy.ROSInterruptException:
    pass
finally:
    server.terminate()
    server.join()
