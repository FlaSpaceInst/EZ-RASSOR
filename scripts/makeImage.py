#! /usr/bin/python
# Copyright (c) 2015, Rethink Robotics, Inc.

# Using this CvBridge Tutorial for converting
# ROS images to OpenCV2 images
# http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython

# Using this OpenCV2 tutorial for saving Images:
# http://opencv-python-tutroals.readthedocs.org/en/latest/py_tutorials/py_gui/py_image_display/py_image_display.html

import time
# rospy for the subscriber
import rospy
# ROS Image message
from sensor_msgs.msg import Image
# ROS Image message -> OpenCV2 image converter
from cv_bridge import CvBridge, CvBridgeError
# OpenCV2 for saving an image
import cv2

# Instantiate CvBridge
bridge = CvBridge()

def image_callback(msg):
    print("Working")
    try:
        # Convert your ROS Image message to OpenCV2
        cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError, e:
        print("Error")
    else:
        # Save your OpenCV2 image as a jpeg 
        cv2.imwrite('camera_image.jpeg', cv2_img)
	time.sleep(1)

def image_callback_2(msg):
    try:
        # Convert your ROS Image message to OpenCV2
        cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError, e:
        print("Error")
    else:
        # Save your OpenCV2 image as a jpeg 
        cv2.imwrite('camera_image_2.jpeg', cv2_img)
	time.sleep(1)

def image_callback_3(msg):
    try:
        # Convert your ROS Image message to OpenCV2
        cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError, e:
        print("Error")
    else:
        # Save your OpenCV2 image as a jpeg 
        cv2.imwrite('camera_image_3.jpeg', cv2_img)
	time.sleep(1)

def image_callback_4(msg):
    print("Works")
    try:
        # Convert your ROS Image message to OpenCV2
        cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError, e:
        print("Error")
    else:
        # Save your OpenCV2 image as a jpeg 
        cv2.imwrite('camera_image_4.jpeg', cv2_img)
	time.sleep(1)

def main():
    rospy.init_node('image_listener')
    # Define your image topic
    image_topic = "/ez_rassor/front_camera/left/image_raw"
    image_topic_2 = "/ez_rassor/front_camera/right/image_raw"
    image_topic_3 = "/ez_rassor/back_camera/right/image_raw"
    image_topic_4 = "/ez_rassor/back_camera/left/image_raw"
    # Set up your subscriber and define its callback
    rospy.Subscriber(image_topic, Image, image_callback)
    rospy.Subscriber(image_topic_4, Image, image_callback_2)
    rospy.Subscriber(image_topic_3, Image, image_callback_3)
    rospy.Subscriber(image_topic_4, Image, image_callback_4)
    # Spin until ctrl + c
 
    rospy.spin()

if __name__ == '__main__':
    main()
