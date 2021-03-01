#!/usr/bin/env python

import rospy
import math
import numpy as np

# NOTE: Imported 'UpdateRoverStatus' here
from ezrassor_swarm_control.srv import GetRoverStatus, PreemptPath, UpdateRoverStatus


def euclidean_distance(x1, x2, y1, y2):
    """ Calculate Euclidean distance from (x1,y1) to (x2,y2). """

    return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)


def euclidean2D(a, b):
    """
    Returns the 2D (x and y axes) euclidean distance between 2 ROS Points
    """

    return np.sqrt((b.x - a.x) ** 2 + (b.y - a.y) ** 2)

# Client node
def get_rover_status(rover_num):
    """
    ROS service that allows the swarm controller to request and
    receive a given rover's current position, battery, and activity
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
        print("Service call failed: {}".format(e))

# TODO: Create a ROS service that updates a rovers world state activity
def update_rover_status(rover_num, new_activity):
    """
    ROS service that allows the swarm controller to update a 
    rovers status (i.e, activity, battery, etc.)
    """

    # Build the service endpoint
    service = "/ezrassor{}/update_rover_status".format(rover_num)

    # Ensure it's running
    rospy.wait_for_service(service, 5.0)

    try:
        # Retrieve updated rover status 
        update_status = rospy.ServiceProxy(service, UpdateRoverStatus) # Service call
        response = update_status(str(new_activity)) # Service response



        return response

    except rospy.ServiceException as e:
         print("Service call failed: {}".format(e))



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
        print("Service call failed: {}".format(e))
