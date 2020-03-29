#!/usr/bin/env python

import rospy
import math
from geometry_msgs.msg import Point

from ezrassor_swarm_control.srv import GetRoverStatus

def euclidean_distance(x1, x2, y1, y2):
    """ Calculate Euclidean distance from (x1,y1) to (x2,y2). """
    
    return math.sqrt( (x2-x1)**2 + (y2-y1)**2 )

def get_rover_status(rover_num):
    """
    ROS service that allows the swarm controller to request and
    receive a given rover's current position
    """

    # Build the service endpoint
    service = '/ezrassor{}/rover_status'.format(rover_num)

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