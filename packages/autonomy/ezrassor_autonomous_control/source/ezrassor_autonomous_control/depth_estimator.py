#!/usr/bin/env python

import rospy
import math
import numpy as np
from numpy import inf, nan
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import LaserScan
import std_msgs
from std_msgs.msg import Int8
import time

# Obstacle Detection
def obst_detect(data):
    pub = rospy.Publisher('obstacle_detect', Int8, queue_size=10)

    """Set thresholds."""
    if np.nanmin(data.ranges) < 1.0:
        rospy.logdebug("MOVE BACKWARD!")
        pub.publish(3)
    else:
        rospy.logdebug("MOVE FORWARD!")
        pub.publish(0)

def depth_estimator():
    rospy.init_node('depth_estimator')
    rospy.Subscriber("scan", LaserScan, obst_detect)
    rospy.loginfo("Depth estimator initialized.")
    rospy.spin()

if __name__ == "__main__":
    try:
        depth_estimator()
    except:
        pass
