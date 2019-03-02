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
# The following code assumes Gazebo to be running and the EZ-RASSOR executing the following scripts:
#
#**************************************************************************************************
#
#   ROS_NAMESPACE=ez_rassor/front_camera rosrun stereo_image_proc stereo_image_proc
#
#   rosrun image_view stereo_view stereo:=ez_rassor/front_camera image:=image_rect
#
#**************************************************************************************************
#    These scripts utilize the Disparity map generation provided by ROS out of the box using the 
#    stereo_image_proc package.  So far this has returned the fastest disparity map generation.  
#==================================================================================================

# Commands to be sent to AI Control. 
commands = {
    'forward' : 0b100000000000, 'reverse' : 0b010000000000, 'left' : 0b001000000000, 'right' : 0b000100000000
}

# Obstacle Detection
def obst_detect(data):

    pub = rospy.Publisher('ez_rassor/obstacle_detect', Int8, queue_size=10)

    """Set thresholds.""" 
    if data[RIGHT].min() > data[LEFT].min() and data[LEFT].min() < 1:
        print("MOVE RIGHT!")
        pub.publish(commands['forward'])
    elif data[LEFT].min() > data[RIGHT].min() and data[RIGHT].min() < 1:
        print("MOVE LEFT!")
        pub.publish(commands['left'])
    else:
         print("MOVE FORWARD!")
         pub.publish(commands['right'])


# 
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
    div_mat = np.split(mean_pool, 2, axis=1)

    """Perform obstacle detection."""
    obst_detect(div_mat)


def depth_estimator():
    rospy.init_node('depth_estimator')
    rospy.Subscriber("/ez_rassor/front_camera/disparity", DisparityImage, callback)
    rospy.spin()

if __name__ == "__main__":
    try:
        depth_estimator()
    except:
        pass