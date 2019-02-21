#!/usr/bin/env python
import rospy
from std_msgs.msg import Int8, Int16, String
from stereo_msgs.msg import DisparityImage
import time
import math

def odometryCallBack



def depth_estimator():
	rospy.Subscriber("/ez_rassor/front_camera/disparity", DisparityImage, odometryCallBack)
	rospy.spin()

if __name__ == "__main__":
	try:
		depth_estimator()
	except ROSInterruptException:
        pass