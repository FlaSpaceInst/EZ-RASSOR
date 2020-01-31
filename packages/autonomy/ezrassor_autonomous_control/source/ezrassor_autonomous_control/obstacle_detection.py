#!/usr/bin/env python

import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, LaserScan, CameraInfo
import numpy as np
import math
import image_geometry
import time

angle_min = None
angle_max = None
angle_increment = None
frame_id = None
ranges_size = None
scan_time = None
range_min = None
range_max = None
floor_height = 0.08
min_hole_height = 0.05
floor_error = 1.05

farthest_point_pub = rospy.Publisher('obstacle_detection/farthest_point',
                                        LaserScan, queue_size = 10)
floor_projection_pub = rospy.Publisher('obstacle_detection/floor_proj',
                                        LaserScan, queue_size = 10)
holes_pub = rospy.Publisher('obstacle_detection/holes',
                                        LaserScan, queue_size = 10)

""" Initializes data for LaserScan messages

Analyzes a given CameraInfo message to determine the fields of the LaserScan
message (except for the ranges array, which is determined each time a new point
cloud is received). This method, as well as the "angle_between_rays" method,
is based heavily on the C++ source code of the "depthimage_to_laserscan"
package.
"""
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

    global angle_max, angle_min, angle_increment, frame_id, ranges_size
    ranges_size = width
    frame_id = camera_info.header.frame_id
    angle_max = angle_between_rays(left_ray, center_ray)
    angle_min = -angle_between_rays(center_ray, right_ray)
    angle_increment = (angle_max-angle_min) / (ranges_size-1)

"""Finds the angle (in radians) between two 3D rays."""
def angle_between_rays(ray1, ray2):
    dot_product = ray1[0]*ray2[0] + ray1[1]*ray2[1] + ray1[2]*ray2[2]
    magnitude1 = math.sqrt(((ray1[0]**2) + (ray1[1]**2) + (ray1[2]**2)))
    magnitude2 = math.sqrt(((ray2[0]**2) + (ray2[1]**2) + (ray2[2]**2)))
    return math.acos(dot_product / (magnitude1 * magnitude2))

"""Creates and returns a LaserScan object based on the given ranges list."""
def create_laser_scan(ranges):
    scan = LaserScan()
    scan.header.stamp = rospy.Time.now()
    scan.header.frame_id = frame_id
    scan.angle_min = angle_min
    scan.angle_max = angle_max
    scan.angle_increment = angle_increment
    scan.time_increment = 0.0
    scan.scan_time = scan_time
    scan.range_min = range_min
    scan.range_max = range_max
    scan.ranges = ranges
    scan.intensities = []
    return scan

""" Converts PointCloud2 to LaserScan based on Farthest Point method

Given a PointCloud2 message representing the floor, this method uses the
Farthest Point method proposed by Ghani et al. in "Detecting negative
obstacle using Kinect sensor" to create a LaserScan containing the farthest
point the robot can see in each direction. This LaserScan is useful for
detecting cliffs or deep holes that the robot cannot see past.
"""
def hole_detection(point_cloud):
    start_total = time.time()
    far_ranges = [float("nan")] * ranges_size
    proj_ranges = [float("nan")] * ranges_size

    start = time.time()
    # read points directly from point cloud message
    pc = np.frombuffer(point_cloud.data, np.float32)
    # reshape into array of xyz values
    pc = np.reshape(pc, (-1, 8))[:, :3]
    # remove NaN points
    pc = pc[~np.isnan(pc).any(axis=1)]
    rospy.loginfo("Read pc: {}".format(str(time.time() - start)))

    if pc.size > 0:
        start = time.time()
        farthest_point(far_ranges, pc)
        rospy.loginfo("farthest point: {}".format(str(time.time() - start)))
        start = time.time()
        floor_projection(proj_ranges, pc)
        rospy.loginfo("floor project: {}".format(str(time.time() - start)))

        min_ranges = [np.nanmin((x, y)) for (x, y) in zip(far_ranges, proj_ranges)]

        farthest_point_pub.publish(create_laser_scan(far_ranges))
        floor_projection_pub.publish(create_laser_scan(proj_ranges))
        holes_pub.publish(create_laser_scan(min_ranges))
    rospy.loginfo("total: {}".format(str(time.time() - start_total)))

def to_laser_scan(forward, right):
    angles = np.arctan2(right, forward)
    steps = np.divide(np.subtract(angles, angle_min), angle_increment).astype(int)
    dists = np.sqrt(np.add(np.square(forward), np.square(right)))
    return steps, dists

def farthest_point(ranges, pc):
    forward = pc[:,2]
    right = pc[:,0]
    steps, dists = to_laser_scan(forward, right)
    for step, dist in zip(steps, dists):
        if math.isnan(ranges[step]) or dist > ranges[step]:
            ranges[step] = dist

def floor_projection(ranges, pc):
    threshold = floor_height * floor_error + min_hole_height
    filtered_pc = pc[pc[:,1] > threshold]

    if filtered_pc.size > 0:
        forward = filtered_pc[:,2]
        down = filtered_pc[:,1]
        right = filtered_pc[:,0]

        forward_projs = np.divide(np.multiply(forward, floor_height), down)
        right_projs = np.divide(np.multiply(right, forward_projs), forward)

        steps, dists = to_laser_scan(forward_projs, right_projs)

        for step, dist in zip(steps, dists):
            if math.isnan(ranges[step]) or dist < ranges[step]:
                ranges[step] = dist

"""Initializes obstacle detection."""
def obstacle_detection(scan_time=1./30, range_min=0.105, range_max=10.):
    rospy.init_node('obstacle_detection')
    rospy.loginfo("Obstacle Detection initialized.")
    globals()['scan_time'] = scan_time
    globals()['range_min'] = range_min
    globals()['range_max'] = range_max
    camera_info = rospy.wait_for_message("depth/camera_info", CameraInfo)
    init_laserscan(camera_info)
    rospy.Subscriber("depth/points", PointCloud2, hole_detection)
    rospy.spin()

if __name__ == "__main__":
    try:
        obstacle_detection()
    except:
        pass
