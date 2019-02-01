#/usr/bin/env python

import rospy
import cv2
import numpy as np
import message_filters
from matplotlib import pyplot as plt
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import time
import scipy


class Rectify:

	def __init__(self):
		
		self.bridge = CvBridge()
		
		# Filter messages so they can be TimeSynced
		self.imgL_sub = message_filters.Subscriber("ez_rassor/front_camera/left/image_raw")
		self.imgR_sub = message_filters.Subscriber("ez_rassor/front_camera/right/image_raw")
		self.ts = message_filters.TimeSynchronizer([self.imgL_sub, self.imgR_sub], 1)

	def callback(data):
		rospy.loginfo(rospy.get_caller_id() + "I can SEE! %d", data.height)


	def disp_test():

		rospy.init_node('listener', anonymous=True)

		sub = rospy.Subscriber("/ez_rassor/front_camera/right/image_raw", Image, callback)

		#imgL = cv2.imread('left.jpeg', 0)
		#imgR = cv2.imread('right.jpeg', 0)

		#stereo = cv2.createStereoBM(numDisparities=16, blockSIze=15)
		#disparity = stereo.compute(imgL, imgR)
		#plt.imshow(disparity, 'gray')
		#plt.show()
		rospy.spin()


if __name__ == "__main__":
	try:
		disp_test()
	except rospy.ROSInterruptException:
		pass
