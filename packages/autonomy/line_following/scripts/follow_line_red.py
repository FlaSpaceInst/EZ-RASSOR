#!/usr/bin/env python
# Written by Christopher Taliaferro

import rospy
import cv2
import numpy as np 
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from std_msgs.msg import Float64
from gazebo_msgs.msg import LinkStates
from sensor_msgs.msg import JointState


class LineFollowerRed(object):

    def __init__(self):

        self.bridge_object = CvBridge()
        self.image_sub = rospy.Subscriber("/ez_rassor/front_camera/left/image_raw",Image,self.camera_callback)
        self.line_error_pub = rospy.Publisher("/ez_rassor/track_following", Float64, queue_size=100)

    def camera_callback(self,data):

        try:
            cv_image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)

        height, width, channels = cv_image.shape
        descentre = 100
        rows_to_watch = 20
        crop_img = cv_image[(height)/2+descentre:(height)/2+(descentre+rows_to_watch)][1:width]

        # Convert from RGB to HSV
        hsv = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)

        # Define Colors in HSV
        upper_red = np.array([5,255,255])
        lower_red = np.array([0,100,100])

        # Threshold the HSV image to get only (in this case) red colors
        mask = cv2.inRange(hsv, lower_red, upper_red)

        # Calculate centroid of the blob on the binary image using ImageMoments
        m = cv2.moments(mask, False)
        try:
            cx, cy = m['m10']/m['m00'], m['m01']/m['m00']
        except ZeroDivisionError:
            cx, cy = height/2, width/2

        # Bitwise AND mask and origional image
        res = cv2.bitwise_and(crop_img, crop_img, mask= mask)

        # Draw the centroid in the resultant image
        cv2.circle(res,(int(cx), int(cy)), 10,(255,255,0),-1)

        cv2.imshow("Starting Image", cv_image)
        cv2.imshow("HSV", hsv)
        cv2.imshow("MASK", mask)
        cv2.imshow("RES", res)

        cv2.waitKey(1)


        # Calculate movement needed
        error_x = cx - width / 2;
        twist_object = Twist();
        twist_object.linear.x - 0.2;
        twist_object.angular.z = -error_x / 100;
        rospy.loginfo("Angular Value Sent ===>"+str(twist_object.angular.z))
        self.line_error_pub.publish(float(twist_object.angular.z))
 


def clean_up(self):
    cv2.destroyAllWindows()

def main():
    rospy.init_node('line_following_node', anonymous = True)

    line_follower_object = LineFollowerRed()

    rate = rospy.Rate(30)
    ctrl_c = False
    def shutdownhook():
        line_follower_object.clean_up()
        rospy.loginfo("Shutting Down")
        ctrl_c = True

    rospy.on_shutdown(shutdownhook)

    while not ctrl_c:
        rate.sleep()

if __name__ == '__main__':
    main()
