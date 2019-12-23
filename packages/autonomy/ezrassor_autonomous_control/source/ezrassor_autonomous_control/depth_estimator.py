#!/usr/bin/env python

import rospy
import math
import numpy as np
from numpy import inf, nan
import cv2
from cv_bridge import CvBridge, CvBridgeError
from stereo_msgs.msg import DisparityImage
import std_msgs
from std_msgs.msg import Int8
import time

LEFT = 0
RIGHT = 1

# Written by Tyler Duncan

# Obstacle Detection
def obst_detect(data):

    pub = rospy.Publisher('obstacle_detect', Int8, queue_size=10)

    """Set thresholds."""
    if data[RIGHT].min() > data[LEFT].min() and data[LEFT].min() < 1.5:
        rospy.logdebug("MOVE RIGHT!")
        pub.publish(1)
    elif data[LEFT].min() > data[RIGHT].min() and data[RIGHT].min() < 1.5:
        rospy.logdebug("MOVE LEFT!")
        pub.publish(2)
    elif data[LEFT].min() < 1 and data[RIGHT].min() < 1.5:
        rospy.logdebug("MOVE BACKWARD!")
        pub.publish(3)
    else:
        rospy.logdebug("MOVE FORWARD!")
        pub.publish(0)


def callback(data):

    """Convert disparity image message to Numpy matrix"""
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(data.image, "8UC1").astype("float64")

    """Convert disparity values to distance values.

    Using this equation:

                                Z[i][j] = fT / d[i][j]

    Where f is the focal distance, T is the baseline distance, d is the disparity value
    at postion (i, j), and Z is distance from the camera to the pixel (i, j).

    Every entry in the disparity matrix must inverted to it's reciprocal.
    Then multiplied by both the focal length of both cameras, f, and the baseline
    distance between the cameras, T.
    This is done by performing a Hadamard Product (element-wise) on the disparty
    matrix.
    """

    depth_mat = np.multiply((data.f * data.T), np.reciprocal(cv_image.astype("float64")))

    """Remove all infinite values and nan values from distance matrix that may be present."""
    depth_mat[depth_mat == inf] = 1000
    depth_mat[depth_mat == nan] = 1000

    """Initialize values for pooling."""
    M, N = depth_mat.shape
    K = 8
    L = 8
    MK = M // K
    NL = N // L

    """Perform mean pooling on distance matrix to minimize noise."""
    mean_pool = depth_mat[:MK*K, :NL*L].reshape(MK, K, NL, L).mean(axis=(1,3))

    """Divide distance matrix into two vertical columns."""
    div_mat = np.split(mean_pool[20], 2, axis=0)
    # print(div_mat)

    """Perform obstacle detection."""
    obst_detect(div_mat)


def depth_estimator():
    rospy.init_node('depth_estimator')
    rospy.Subscriber("disparity", DisparityImage, callback)
    rospy.loginfo("Depth estimator initialized.")
    rospy.spin()

if __name__ == "__main__":
    try:
        depth_estimator()
    except:
        pass
