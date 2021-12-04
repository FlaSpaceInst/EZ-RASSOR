# Created by Christopher Jackson and Robert Forristall

#!/usr/bin/env python3
import pyrealsense2 as rs
import numpy as np
import cv2
from cv_bridge import CvBridge
import tensorflow as tf
import rospy
import rospkg
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Float32, Float64MultiArray
import roslaunch


class ImageHolder:
    def __init__(self):
        self.image = None
        self.depth_image = None
        self.camera_info = None
        self.image_height = 0
        self.image_width = 0
        self.depth_image_height = 0
        self.depth_image_width = 0
        self.encoding = None
        self.cv_bridge = CvBridge()
        # self.launch = roslaunch.scriptapi.ROSLaunch()
        # self.launch.start()

        self.location_pub = rospy.Publisher(
            "object_detection/paver_location", Float64MultiArray, queue_size=10
        )

    def SetImage(self, data):
        self.image = self.cv_bridge.imgmsg_to_cv2(
            data, desired_encoding="passthrough"
        )
        self.image_height = data.height
        self.image_width = data.width
        self.encoding = data.encoding

    def GetImage(self):
        return self.image

    def SetDepthImage(self, data):
        self.depth_image = self.cv_bridge.imgmsg_to_cv2(
            data, desired_encoding="passthrough"
        )
        self.depth_image_height = data.height
        self.depth_image_width = data.width

    def GetDepthImage(self):
        return self.depth_image

    def GetImageSize(self):
        return (self.image_width, self.image_height)

    def SetInfo(self, data):
        self.camera_info = data

    def GetInfo(self):
        return self.camera_info

    # Function for converting pixel coordinates to physical cartesian coordinates
    def ConvertToPhysCoords(self, x, y, depth, cameraInfo):
        _intrinsics = rs.intrinsics()
        _intrinsics.width = cameraInfo.width
        _intrinsics.height = cameraInfo.height
        _intrinsics.ppx = cameraInfo.K[2]
        _intrinsics.ppy = cameraInfo.K[5]
        _intrinsics.fx = cameraInfo.K[0]
        _intrinsics.fy = cameraInfo.K[4]
        _intrinsics.model = rs.distortion.none

        # If the camera has D values, the following line must be used, otherwise, the coeffs can be set to an array of zeros
        # _intrinsics.coeffs = [i for i in cameraInfo.D]
        _intrinsics.coeffs = [0, 0, 0, 0, 0]

        result = rs.rs2_deproject_pixel_to_point(_intrinsics, [x, y], depth)

        return -result[0], -result[1], result[2]

    def ObjectDetection(self, data):
        rospy.loginfo("Starting Object Detection")
        rospack = rospkg.RosPack()

        # Frozen weights and configurations for our model
        PATH_TO_FIG = (
            rospack.get_path("ezrassor_arm_autonomous_control")
            + "/source/ezrassor_arm_autonomous_control/frozen_inference_graph.pb"
        )
        PATH_TO_TXT = (
            rospack.get_path("ezrassor_arm_autonomous_control")
            + "/source/ezrassor_arm_autonomous_control/faster_rcnn_inception_v2_coco_2018_01_28.pbtxt"
        )

        # Load in the model with it's fozen weights and configurations
        cvNet = cv2.dnn.readNetFromTensorflow(PATH_TO_FIG, PATH_TO_TXT)

        # Get the color and depth images from the camera convert them to arrays with values
        color_frame = self.GetImage()
        color_image = np.asanyarray(color_frame)
        depth_frame = self.GetDepthImage()
        depth_image = np.asanyarray(depth_frame)
        rows = color_image.shape[0]
        cols = color_image.shape[1]

        # Set the input of the model to the color image and run the model with our input
        cvNet.setInput(
            cv2.dnn.blobFromImage(
                color_image, size=(300, 300), swapRB=True, crop=False
            )
        )
        cvOut = cvNet.forward()

        left = 0
        right = 0
        top = 0
        bottom = 0
        pointX = 0
        pointY = 0
        distance = 0
        scoreMax = 0.0

        # Loop through objects our model detected and save the coordinates of the object with the best confidence score
        for detection in cvOut[0, 0, :, :]:
            score = float(detection[2])
            if score > scoreMax:
                left = detection[3] * cols
                top = detection[4] * rows
                right = detection[5] * cols
                bottom = detection[6] * rows

        rospy.loginfo("scoreMax = " + str(scoreMax))

        # Draw the bounding box on our color image
        cv2.rectangle(
            color_image,
            (int(left), int(top)),
            (int(right), int(bottom)),
            (0, 255, 0),
            thickness=2,
        )

        # Calculate the center of the bounding box for getting the most accurate depth of the paver
        pointX = (int)(left + right) / 2
        pointY = (int)(bottom + top) / 2
        rospy.loginfo("pointX = " + str(pointX))
        rospy.loginfo("pointY = " + str(pointY))

        # Get the value from our depth image at the center pixel coordinates we calculated
        distance = depth_image[pointY, pointX]
        rospy.loginfo("distance = " + str(distance))

        cv2.putText(
            color_frame,
            "{}mm".format(distance),
            (pointX, pointY),
            cv2.FONT_HERSHEY_PLAIN,
            2,
            (0, 255, 0),
            2,
        )

        # Convert our pixel coordinates to physical cartesian coordinates
        (xCoord, yCoord, zCoord) = self.ConvertToPhysCoords(
            pointX, pointY, distance, self.GetInfo()
        )
        rospy.loginfo("xCoord = " + str(xCoord))
        rospy.loginfo("yCoord = " + str(yCoord))
        rospy.loginfo("zCoord = " + str(zCoord))

        # publish the physical coordinates for the autonomous controller to determine where the arm needs to be moved
        pub_arr = Float64MultiArray()
        pub_arr.data = [xCoord, yCoord, zCoord]
        self.location_pub.publish(pub_arr)

        # cv2.imshow('RealSense', color_image)
        # cv2.waitKey()

    def DemoObjectDetection(self, data):
        # self.spawnPaver([1.5, 0.1, 0.1])
        rospy.sleep(rospy.Duration(secs=2))
        self.ObjectDetection(1)


def onStart(camera_image_topic, camera_info_topic, command_topic):
    try:
        rospy.init_node("object_detection")
        holder = ImageHolder()
        rospy.Subscriber(
            "/ezrassor1/color/image_raw_arm", Image, holder.SetImage
        )
        rospy.Subscriber(
            "/ezrassor1/depth/image_raw_arm", Image, holder.SetDepthImage
        )
        rospy.Subscriber(
            "/ezrassor1/color/camera_info_arm", CameraInfo, holder.SetInfo
        )
        rospy.Subscriber(command_topic, Float32, holder.ObjectDetection)
        rospy.sleep(rospy.Duration(secs=2))
        # holder.spawnPaver([2.0, 0.0, 0.0])
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
