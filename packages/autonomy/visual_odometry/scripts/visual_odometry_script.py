#!/usr/bin/env python
import rospy
from std_msgs.msg import Int8, Int16, String
import time
import math
import numpy as np
import cv2
from matplotlib import pyplot as plt
from sensor_msgs.msg import Image
import std_msgs
import message_filters
from stereo_msgs.msg import DisparityImage
from cv_bridge import CvBridge, CvBridgeError


# This function will gather the disparity maps and camera feeds from two different times. It will output the angle of rotation and a translation vector between the two locations. Its inputs should be the camera image and disparity image AFTER the time has elapsed. It will gather the previous images.
def odometryCallBack(later_camera, later_disparity):

	# CVBridge allows us to change ROS images to images opencv is able to use
	bridge = CvBridge()

	# Make the later image usable by opencv
	camera_later_cv = bridge.imgmsg_to_cv2(later_camera, "bgr8")

	# Make the disparity image usable by opencv
	disparity_later_cv = bridge.imgmsg_to_cv2(later_disparity.image, "8UC1").astype("float64")
	
	# Get the earlier camera image (now with key points labelled on it) and the earlier disparity image
	camera_early_kp, disparity_early = return_images('camera.png','disparity.png')

	# This function is meant to test that the images are indeed separated by a seven second gap. This is done by printing all of the images in this callback to new files, to see the difference. The main thing we are looking for is 1.png and 2.png to be different if the robot has moved.
	testTimeGap(camera_early_kp, camera_later_cv, disparity_early, disparity_later_cv)

	# !!!!!!!!!!!!------ INSERT LUCAS-KANADE CODE HERE --------!!!!!!!!!!!!!!!#	

	# !!!!!!!!!!!!------ INSERT TRIANGULATION CODE HERE --------!!!!!!!!!!!!!!!#

	# !!!!!!!!!!!!------ INSERT TRANSLATION-ANGLE VECTOR CODE HERE --------!!!!!!!!!!!!!!!#	
 	
	# Save the current images as the old images now
	save_images(camera_later_cv, disparity_later_cv)

	# Current means of delaying the time between images. It will be lower in practice. But may be a bottleneck until we can figure out how to avoid saving the images as pngs since the imwrite eats up time.	
	print("Kill or adjust now to view images")
	rospy.sleep(7)
	print("Leave be")
	

# This is the main function
def depth_estimator():
	# Initialize the listener
	rospy.init_node('image_listener')

	# The locations of the needed topics for the camera feed and the disparity feed
	image_topic = "/ez_rassor/front_camera/right/image_raw"
	disparity_topic = "/ez_rassor/front_camera/disparity"

	# Subscribe to their respective topics
	camera = message_filters.Subscriber(image_topic, Image)
	disparity = message_filters.Subscriber(disparity_topic, DisparityImage)

	# We will now synchronize the times of these two feeds before performing the callback. This ensures the camera Image we get lines up with the disparity image we get. The ten at the end is the size of the queue of messages the synchronizer can receive while waiting for matching timestamps of the two images
	synched = message_filters.TimeSynchronizer([camera,disparity], 10)

	# This callback will gather the saved images from the previous callback AND take in the current images and set it up for use by the LKT algorithm
	synched.registerCallback(odometryCallBack)

	# Keeps this node going until killed
	rospy.spin()

# This function will return two saved images with the names it is passed as arguments.
def return_images(first_name, second_name):
	
	# Read in the pictures
	camera = cv2.imread(first_name, 0)
	disparity = cv2.imread(second_name, 0)
	
	# Return them
	return camera, disparity

# This function will take in a camera Image and a disparity Image and save them as a png to be used later
def save_images(camera, disparity):

	# Perform the FAST feature detection algorithm on the image before saving it.
	camera_key_points = fast(camera)

	# Once we have a picture from the camera with detected features, we will save it as a png for later use
	cv2.imwrite('camera.png', camera_key_points)

	cv2.imwrite('disparity.png', disparity)

# This function will perform the FAST feature detector on an image and return an image with the features already detected on it. It will perform it WITH NonmaxSuppression
def fast(image):

	# Create the feature detector first
	fast_detector = cv2.FastFeatureDetector_create()
	
	# If nonmaxmimumsuppression is not wanted, then uncomment the comment below
	# fast_detector.setNonmaxSuppression(0)

	# Actually perform the feature detection. It will return a bunch of keypoints
	key_points = fast_detector.detect(image, None)

	# Draw the keypoints over the original image now
	key_points_plus_image = cv2.drawKeypoints(image, key_points, None, color=(255,0,0))

	# Return the image with keypoints on it
	return key_points_plus_image

# This function saves four given images as pngs to be checked manually after the program finishes
def testTimeGap(image1, image2, image3, image4):
	
	cv2.imwrite("Camera_Old.png", image1)
	cv2.imwrite("Camera_New.png", image2)
	cv2.imwrite("New_Disparity.png", image3)
	cv2.imwrite("Old_Disparity.png", image4)

if __name__ == "__main__":
	
	# Run the main function
	depth_estimator()
