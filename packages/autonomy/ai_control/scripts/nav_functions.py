#!/usr/bin/env python
import rospy
from std_msgs.msg import Int8, Int16, String
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import LinkStates
from sensor_msgs.msg import JointState
from ai_control.msg import ObstacleDetection
from WorldState import WorldState
import time
import math


def euclidean_distance(x1, x2, y1, y2):
    """ Calculate Euclidean distance from (x1,y1) to (x2,y2). """
    
    return math.sqrt( (x2-x1)**2 + (y2-y1)**2 )