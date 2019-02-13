#!/usr/bin/env python

import rospy
import numpy as np 
import cv2
from cv_bridge import CvBridge, CvBridgeError 
from stereo_msgs.msg import DisparityImage
import std_msgs

# TODO: I need to figure out how to get the disparityImage message in such a way 
# 		that I can use the focal length value, baseline value, and disparity matrix
#		values to calculate a distance matrix. 
#		
#		Z[i][j] = fT / d[i][j], where
#		
#		-	Z[i][j] = the distance to pixel (i, j) 
#		-	f = focal distance 
#		-	T = baseline 
#		-	d = disparity value of pixel (i, j). 

#		Numpy.reciprocal will invert all entries in the disparity matrix and then we simply need to
#		multiply by fT on each entry to get depth information. 

def callback(data):
	# Testing Subscription. 
	# rospy.loginfo(rospy.get_caller_id() + "The camera focal length is " + str(data.f))


	bridge = CvBridge()
	cv_image = bridge.imgmsg_to_cv2(data.image, "8UC1")
	
	depth_mat = np.multiply((data.f * data.T), np.reciprocal(cv_image))
	max_arr = depth_mat.max(0)
	rows, cols = depth_mat.shape
	
	# max_left = max_arr[0:320]
	# max_right = max_arr[321:639]

	# if max_left.mean > max_right.mean:
	# 	print("MOVE RIGHT!")
	# elif max_left.mean < max_right.mean:
	# 	print("MOVE LEFT!") 

def depth_estimator():

	rospy.init_node('depth_estimator')
	rospy.Subscriber("/ez_rassor/front_camera/disparity", DisparityImage, callback)
	rospy.spin()

if __name__ == "__main__":
	try:
		depth_estimator()
	except ROSInterruptException:
		pass