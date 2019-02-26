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
#	These scripts utilize the Disparity map generation provided by ROS out of the box using the 
#	stereo_image_proc package.  So far this has returned the fastest disparity map generation.  



#==================================================================================================
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
#
#		Numpy.reciprocal will invert all entries in the disparity matrix and then we simply need to
#		multiply by fT on each entry to get depth information. 
#===================================================================================================

def obst_detect(data):
	pub = rospy.Publisher('/obst_detect', Int8, queue_size=10)

	# Bit Strings for future use. 
	move_forward = 0b00000001
	move_backward = 0b01000000
	move_left = 0b00100000
	move_right = 0b00010000
	stop = 0b00001000
	stop_left = 0b00000100
	stop_right = 0b00000010
	

	# If an object is less than 1 meter away, avoid it. 
	if data[1].min() > data[0].min() and data[0].min() < 1:
		print("MOVE RIGHT!")
		pub.publish(1)
	elif data[0].min() > data[1].min() and data[1].min() < 1:
		print("MOVE LEFT!")
		pub.publish(2)
	else:
	 	print("MOVE FORWARD!")
	 	pub.publish(0)

	# USE THIS IF/WHEN WE GET MORE COMPLICATED AUTO MANEUVERS 
	#
	# if data[1].min() > data[0].min() and 0.7 < data[0].min() < 3:
	# 	print("MOVE RIGHT!")
	# 	pub.publish(move_right)
	# elif data[0].min() > data[1].min() and 0.7< data[1].min() < 3:
	# 	print("MOVE LEFT!")
	# 	pub.publish(move_left)
	# elif data[0].min() <= 0.7 and data[1].min() <= 0.7:
	# 	print("MOVE BACK!")
	# 	pub.publish(move_backward)
	# elif data[0].min() <= 0.7 and data[1].min() > 3:
	# 	print("TURN RIGHT IN PLACE!")
	# 	pub.publish(stop_right)
	# elif data[1].min() <= 0.7 and data[0].min() > 3:
	# 	print("TURN LEFT IN PLACE!")
	# 	pub.publish(stop_left)
	# else:
	#  	print("MOVE FORWARD!")
	#  	pub.publish(move_forward)

	

def callback(data):
	# Testing Subscription. 
	# rospy.loginfo(rospy.get_caller_id() + "The camera focal length is " + str(data.f))

	start = time.time()
	bridge = CvBridge()
	cv_image = bridge.imgmsg_to_cv2(data.image, "8UC1").astype("float64")
	
	depth_mat = np.multiply((data.f * data.T), np.reciprocal(cv_image.astype("float64")))

	depth_mat[depth_mat == inf] = 1000
	depth_mat[depth_mat == nan] = 1000

	# print(depth_mat)

	M, N = depth_mat.shape

	K = 8
	L = 8

	MK = M // K
	NL = N // L

	mean_pool = depth_mat[:MK*K, :NL*L].reshape(MK, K, NL, L).mean(axis=(1,3))
	
	
	div_mat = np.split(mean_pool, 2, axis=1)
	# print(div_mat[2].min())
	end = time.time()
	# print(end - start)

	obst_detect(div_mat)
	# Hello darling!

def depth_estimator():

	rospy.init_node('depth_estimator')
	rospy.Subscriber("/ez_rassor/front_camera/disparity", DisparityImage, callback)
	rospy.spin()

if __name__ == "__main__":
	try:
		depth_estimator()
	except ROSInterruptException:
		pass