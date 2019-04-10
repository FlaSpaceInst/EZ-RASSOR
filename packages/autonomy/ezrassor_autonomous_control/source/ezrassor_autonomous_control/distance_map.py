#!/usr/bin/env python

import rospy
import math
import numpy as np 
from numpy import inf, nan
import cv2
from cv_bridge import CvBridge, CvBridgeError 
from stereo_msgs.msg import DisparityImage
import std_msgs
from std_msgs.msg import Int16
from sensor_msgs.msg import LaserScan
import time

LEFT = 0
RIGHT = 1

# Written by Tyler Duncan

# Offset Laserscan data
def laser_scan_offset(data, increment):
    ranges = []
    number_of_scans = len(data)
    theta = 1.39626
    theta_prime = 2 * np.arctan(theta / 2)
    for i in range(number_of_scans):
        arc_length = data[i] * theta_prime
        new_distance = np.sqrt(((0.5 * arc_length) ** 2) + (data[i] ** 2))
        ranges.append(new_distance)
        if i < number_of_scans / 2:
            theta_prime -= increment * 2
        elif i > number_of_scans / 2:
            theta_prime += increment * 2
        else:
            theta_prime = 0

    return ranges

# Obstacle Detection
def obst_detect(data):

    pub = rospy.Publisher('ezrassor/obstacle_detect', Int16, queue_size=10)
    laser_scan = rospy.Publisher('/scan', LaserScan, queue_size=10)

    scan = LaserScan()

    scan.header.stamp = rospy.Time.now()
    scan.header.frame_id = "camera_depth_frame"
    scan.angle_min = -0.698132
    scan.angle_max = 0.698132
    scan.angle_increment = 0.0174533
    scan.time_increment = 1 / 60
    scan.scan_time = 1 / 60
    scan.range_min = 0
    scan.range_max = 500.0
    scan.ranges = laser_scan_offset((np.append(data[LEFT],data[RIGHT]))[::-1], scan.angle_increment)
    scan.intensities = []

    laser_scan.publish(scan)


    """Set thresholds.""" 
    if data[RIGHT].min() > data[LEFT].min() and data[LEFT].min() < 1.5:
        print("MOVE RIGHT!")
        pub.publish(1)
    elif data[LEFT].min() > data[RIGHT].min() and data[RIGHT].min() < 1.5:
        print("MOVE LEFT!")
        pub.publish(2)
    elif data[LEFT].min() < 1 and data[RIGHT].min() < 1.5:
        print("MOVE BACKWARD!")
        pub.publish(3)
    else:
        print("MOVE FORWARD!")
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
    rospy.Subscriber("/ezrassor/front_camera/disparity", DisparityImage, callback)
    rospy.spin()

if __name__ == "__main__":
    try:
        depth_estimator()
    except:
        pass