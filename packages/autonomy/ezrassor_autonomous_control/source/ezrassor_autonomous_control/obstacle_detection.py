#!/usr/bin/env python

import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, LaserScan, CameraInfo
import numpy as np
import math
import image_geometry
import time

# Coordinate system X, Y, Z = RIGHT, DOWN, FORWARD
XYZ = {
    "RIGHT" : 0,
    "DOWN" : 1,
    "FORWARD" : 2
}

angle_min = None
angle_max = None
angle_increment = None
frame_id = None
ranges_size = None
scan_time = None
range_min = None
range_max = None

threshold = 1.0
camera_height = None
min_hole_depth = None
min_obstacle_height = None

# Publishers
cliffs_pub = rospy.Publisher('obstacle_detection/cliffs',
                                        LaserScan, queue_size = 10)
positive_pub = rospy.Publisher('obstacle_detection/positive',
                                        LaserScan, queue_size = 10)
combined_pub = rospy.Publisher('obstacle_detection/combined',
                                        LaserScan, queue_size = 10)

""" Initializes data for LaserScan messages

Analyzes a given CameraInfo message to determine the fields of the LaserScan
message (except for the ranges array, which is determined each time a new point
cloud is received). This method, as well as the "angle_between_rays" method,
is based heavily on the C++ source code of the "depthimage_to_laserscan"
package.
"""
def init_laserscan(camera_info):
    # Initialize camera 
    cam_model = image_geometry.PinholeCameraModel()
    cam_model.fromCameraInfo(camera_info)
    width = camera_info.width

    # Find vector for leftmost view in camera
    raw_pixel_left = (0, cam_model.cy())
    rect_pixel_left = cam_model.rectifyPoint(raw_pixel_left)
    left_ray = cam_model.projectPixelTo3dRay(rect_pixel_left)

    # Find vector for rightmost view in camera
    raw_pixel_right = (width-1, cam_model.cy())
    rect_pixel_right = cam_model.rectifyPoint(raw_pixel_right)
    right_ray = cam_model.projectPixelTo3dRay(rect_pixel_right)

    # Find vector for center view of camera
    raw_pixel_center = (cam_model.cx(), cam_model.cy())
    rect_pixel_center = cam_model.rectifyPoint(raw_pixel_center)
    center_ray = cam_model.projectPixelTo3dRay(rect_pixel_center)

    # Find the range of angles to be covered in the LaserScan
    global angle_max, angle_min, angle_increment, frame_id, ranges_size
    ranges_size = width
    frame_id = camera_info.header.frame_id
    angle_max = angle_between_rays(left_ray, center_ray)
    angle_min = -angle_between_rays(center_ray, right_ray)

    # Find size of angle buckets in LaserScan
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

""" Converts PointCloud2 to LaserScan

Given a PointCloud2 message representing the area in front of the robot,
this method uses the Farthest Point and Floor Projection methods
proposed by Ghani et al. in "Detecting negative obstacle using Kinect
sensor" to create LaserScans containing the farthest point the robot
can see in each direction (to detect cliffs) and the closest holes
to the robot in each direction. These LaserScans are combined with a
LaserScan containing the closest above-ground obstacles in each
direction to form a LaserScan that contains the closest cliff, hole,
or above-ground obstacle in each direction.
"""
def point_cloud_to_laser_scan(point_cloud):
    # Initial LaserScans assume infinite travel in every direction
    cliff_ranges = [float("nan")] * ranges_size
    positive_ranges = [float("nan")] * ranges_size
    min_ranges = [float("nan")] * ranges_size

    # Read points directly from point cloud message
    pc = np.frombuffer(point_cloud.data, np.float32)
    # Reshape into array of xyz values
    pc = np.reshape(pc, (-1, 8))[:, :3]
    # Remove nan points
    pc = pc[~np.isnan(pc).any(axis=1)]

    # Check if the PointCloud2 has non-nan points
    if pc.size > 0:
        forward = pc[:,XYZ["FORWARD"]]
        right = pc[:,XYZ["RIGHT"]]
        down = pc[:,XYZ["DOWN"]]
        steps, dists = to_laser_scan_data(forward, right)

        # Create matrix where the steps, dists, and down arrays are the columns
        directions = np.column_stack((steps, dists, down))
        # Sort rows by the first column (steps); this is necessary for the next step
        directions = directions[directions[:,0].argsort()]
        # Group rows by step
        directions = np.split(directions, np.unique(directions[:,0], return_index=True)[1][1:],axis=0)

        # Loop through the rows for each step and find obstacles
        for direction in directions:
            # Sort rows for this step by the second column (dist)
            direction = direction[direction[:,1].argsort()]

            # Step is first column of any row
            step = int(direction[0, 0])

            # Since the rows are sorted by dist, the farthest point for this step is in the last row
            cliff_ranges[step] = direction[-1, 1]

            # Slice down and dist in order to perform operations at idx and idx-1
            down1 = direction[:-1, 2]
            down2 = direction[1:, 2]
            dist1 = direction[:-1, 1]
            dist2 = direction[1:, 1]

            # Calculate slope for each pair of points
            drop = np.subtract(down2, down1)

            slope = np.abs(np.divide(drop, np.subtract(dist2, dist1)))

            # Find first index of row where the slope crosses the threshold
            condition = np.logical_or(drop < min_hole_depth, slope > threshold)
            index = condition.argmax() if condition.any() else None

            if index is not None:
                positive_ranges[step] = direction[index, 1]
                min_ranges[step] = min(positive_ranges[step], cliff_ranges[step])
            else:
                min_ranges[step] = cliff_ranges[step]

        cliffs_pub.publish(create_laser_scan(cliff_ranges))
        positive_pub.publish(create_laser_scan(positive_ranges))
        combined_pub.publish(create_laser_scan(min_ranges))

def to_laser_scan_data(forward, right):
    # multiply angles by -1 to get counter-clockwise (right to left) ordering
    angles = np.negative(np.arctan2(right, forward))
    steps = np.divide(np.subtract(angles,angle_min),angle_increment).astype(int)
    # Find the distance each forward, right coordinate from the robot
    dists = np.sqrt(np.add(np.square(forward), np.square(right)))
    return steps, dists

"""Initializes obstacle detection."""
def obstacle_detection(camera_height_yaml=None, min_hole_depth_yaml=None, 
                       min_obstacle_height_yaml=None,  scan_time=1./30, 
                       range_min=0.105, range_max=10.):

    rospy.init_node('obstacle_detection')

    rospy.loginfo("Obstacle Detection initialized.")

    globals()['scan_time'] = scan_time
    globals()['range_min'] = range_min
    globals()['range_max'] = range_max

    globals()['camera_height'] = camera_height_yaml
    globals()['min_hole_depth'] = min_hole_depth_yaml * -1
    globals()['min_obstacle_height'] = min_obstacle_height_yaml

    camera_info = rospy.wait_for_message("depth/camera_info", CameraInfo)

    init_laserscan(camera_info)

    rospy.Subscriber("depth/points", PointCloud2, point_cloud_to_laser_scan)

    rospy.spin()

if __name__ == "__main__":
    try:
        obstacle_detection()
    except:
        pass
