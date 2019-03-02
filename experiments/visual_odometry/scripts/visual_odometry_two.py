#!/usr/bin/env python
import rospy
from std_msgs.msg import Int8, Int16, String
import time
import math
import numpy as np
import cv2
from sensor_msgs.msg import Image
import std_msgs
import message_filters
from stereo_msgs.msg import DisparityImage
from cv_bridge import CvBridge, CvBridgeError

rospy.init_node('ai_control_node', anonymous=True)

images_current = []
images_prev = []
cx = 320
cy = 240
f = 0

rate = rospy.Rate(1)

def LKT(image_current, images_prev):

    lk_params = dict( winSize  = (15,15),
                    maxLevel = 2,
                    criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))

    p0 = FAST(images_prev)

    p1, _, _ = cv2.calcOpticalFlowPyrLK(images_prev, image_current, p0, None, **lk_params)

    return p0, p1


def getPointCloud(points, images, disparity):
    W = np.matrix()
    
    for point in points:
        W = np.vstack([W, [point[0]-cx, point[1]-cy, -f, 1]])

    return W



def triangulationAndTranslation():
    pass

def FAST(image):

	fast_detector = cv2.FastFeatureDetector_create()
	
	key_points = fast_detector.detect(image, None)

	return key_points

def visualOdometryCallback(image_raw, disparity):
    global images_prev, images_current, f
    bridge = CvBridge()

    f = image_raw.f
    image_cv2 = bridge.imgmsg_to_cv2(image_raw, "bgr8")
    disparity_cv2 = bridge.imgmsg_to_cv2(disparity.image, "8UC1").astype("float64")
    

    images_prev = images_current
    images_current = [image_cv2, disparity_cv2]
    

def visual_odometry():
    disparity = message_filters.Subscriber("ez_rassor/front_camera/disparity", DisparityImage)
    image = message_filters.Subscriber("ez_rassor/front_camera/left/image_raw", Image)

    synched = message_filters.TimeSynchronizer([image,disparity], 10)
    synched.registerCallback(visualOdometryCallback)

    rospy.spin()




if __name__ == "__main__":
    try:
        visual_odometry()
    except KeyboardInterrupt:
        print("Error")
