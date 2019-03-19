#!/usr/bin/env python
import rospy
import math


def euclidean_distance(x1, x2, y1, y2):
    """ Calculate Euclidean distance from (x1,y1) to (x2,y2). """
    
    return math.sqrt( (x2-x1)**2 + (y2-y1)**2 )

def calculate_heading(x1, x2, y1, y2):
    """  """

    

