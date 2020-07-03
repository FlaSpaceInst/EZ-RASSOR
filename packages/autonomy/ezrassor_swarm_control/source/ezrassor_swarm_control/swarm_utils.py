#!/usr/bin/env python

import rospy
import math
import numpy as np

from geometry_msgs.msg import Point

from ezrassor_swarm_control.srv import GetRoverStatus, PreemptPath


def euclidean_distance(x1, x2, y1, y2):
    """ Calculate Euclidean distance from (x1,y1) to (x2,y2). """

    return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)


def euclidean2D(a, b):
    """
    Returns the 2D (x and y axes) euclidean distance between 2 ROS Points
    """

    return np.sqrt((b.x - a.x) ** 2 + (b.y - a.y) ** 2)


def get_rover_status(rover_num):
    """
    ROS service that allows the swarm controller to request and
    receive a given rover's current position
    """

    # Build the service endpoint
    service = "/ezrassor{}/rover_status".format(rover_num)

    # Ensure it's running
    rospy.wait_for_service(service, 5.0)

    try:
        # Retrieve rover status
        get_status = rospy.ServiceProxy(service, GetRoverStatus)
        response = get_status()

        # Convert float coordinates to integers
        response.pose.position.x = int(round(response.pose.position.x))
        response.pose.position.y = int(round(response.pose.position.y))
        return response

    except rospy.ServiceException as e:
        print "Service call failed: %s" % e


def preempt_rover_path(rover_num):
    """
    ROS service that allows the swarm controller to force a rover to
    stop following a path
    """

    # Build the service endpoint
    service = "/ezrassor{}/preempt_path".format(rover_num)

    # Ensure it's running
    rospy.wait_for_service(service, 5.0)

    try:
        # Retrieve rover status
        preempt_path = rospy.ServiceProxy(service, PreemptPath)
        response = preempt_path()

        return response

    except rospy.ServiceException as e:
        print "Service call failed: %s" % e
