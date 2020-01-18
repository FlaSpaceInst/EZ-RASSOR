#!/usr/bin/env python

import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, LaserScan, CameraInfo
import numpy as np
import math
import image_geometry

angle_min = None
angle_max = None
angle_increment = None
camera_info = None

farthest_point_pub = rospy.Publisher('/farthest_point_ls', LaserScan, queue_size = 10)

# from the big brains of Jordan and John, and nobody else
def init_laserscan(camera_info):
    cam_model = image_geometry.PinholeCameraModel()
    cam_model.fromCameraInfo(camera_info)
    width = camera_info.width

    raw_pixel_left = (0, cam_model.cy())
    rect_pixel_left = cam_model.rectifyPoint(raw_pixel_left)
    left_ray = cam_model.projectPixelTo3dRay(rect_pixel_left)

    raw_pixel_right = (width-1, cam_model.cy())
    rect_pixel_right = cam_model.rectifyPoint(raw_pixel_right)
    right_ray = cam_model.projectPixelTo3dRay(rect_pixel_right)

    raw_pixel_center = (cam_model.cx(), cam_model.cy())
    rect_pixel_center = cam_model.rectifyPoint(raw_pixel_center)
    center_ray = cam_model.projectPixelTo3dRay(rect_pixel_center)

    angle_max = angle_between_rays(left_ray, center_ray)
    # Negative because the laserscan message expects an opposite rotation of that from the depth image
    angle_min = -angle_between_rays(center_ray, right_ray)

    rospy.loginfo("min angle: " + str(angle_min) + ", max angle: " + str(angle_max))

def angle_between_rays(ray1, ray2):
    dot_product = ray1[0]*ray2[0] + ray1[1]*ray2[1] + ray1[2]*ray2[2]
    magnitude1 = math.sqrt(((ray1[0]**2) + (ray1[1]**2) + (ray1[2]**2)))
    magnitude2 = math.sqrt(((ray2[0]**2) + (ray2[1]**2) + (ray2[2]**2)))
    return math.acos(dot_product / (magnitude1 * magnitude2))

# Obstacle Detection
def obst_detect(point_cloud):
    min_x = min_y = min_z = float("inf")
    max_x = max_y = max_z = float("-inf")
    for p in pc2.read_points(point_cloud, field_names = ("x", "y", "z"), skip_nans=True):
        min_x = min(p[0], min_x)
        min_y = min(p[1], min_y)
        min_z = min(p[2], min_z)
        max_x = max(p[0], max_x)
        max_y = max(p[1], max_y)
        max_z = max(p[2], max_z)

    # rospy.loginfo("min x: " + str(min_x) + ", max x: " + str(max_x) + ", min y: " + str(min_y) + ", max y: " + str(max_y) + ", min z: " + str(min_z) + ", max z: " + str(max_z))

def farthest_point(point_cloud):
    if angle_min is None or angle_max is None or angle_increment is None:
        return

    ranges = []
    for p in pc2.read_points(point_cloud, field_names = ("x", "y", "z"), skip_nans=True):
        forward = p[2]
        down = p[1]
        right = p[0]

        angle = math.atan(right / forward)
        step = (angle - angle_min) // angle_increment
        range =  math.sqrt(((forward**2) + (right**2)))

        if range > ranges[step]:
            ranges[step] = range

    # create laser scan header
    scan = LaserScan()
    scan.header.stamp = rospy.Time.now()
    scan.header.frame_id = frame_id
    scan.angle_min = angle_min
    scan.angle_max = angle_max
    scan.angle_increment = angle_increment
    scan.time_increment = None
    scan.scan_time = None
    scan.range_min = None
    scan.range_max = None
    scan.ranges = ranges
    scan.intensities = []

    # farthest_point_pub.publish()

def depth_estimator():
    rospy.init_node('depth_estimator')
    rospy.loginfo("Depth estimator initialized.")
    camera_info = rospy.wait_for_message("depth/camera_info", CameraInfo)
    init_laserscan(camera_info)
    rospy.Subscriber("depth/points", PointCloud2, obst_detect)
    rospy.spin()

if __name__ == "__main__":
    try:
        depth_estimator()
    except:
        pass
