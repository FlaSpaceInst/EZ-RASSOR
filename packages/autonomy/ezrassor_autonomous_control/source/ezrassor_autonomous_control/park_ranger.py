#!/usr/bin/env python

import rospy
from sensor_msgs.msg import PointCloud2, CameraInfo
import numpy as np
import math
import image_geometry

# **Remove later (used for visualization of local DEM only)
import matplotlib.pyplot as plt

import rospkg
from os import listdir
from os.path import isfile, join
import re

# Coordinate system
XYZ = {
    "RIGHT": 0,
    "DOWN": 1,
    "FORWARD": 2
}

# Used to determine what value to use for a cell in the local DEM when
# multiple points fall into that cell
local_dem_comparison_options = {
    "mean": np.nanmean,
    "median": np.nanmedian,
    "max": np.nanmax,
    "min": np.nanmin
}

point_cloud = None

resolution = None
num_rows = None
num_columns = None
min_row = None
min_column = None

global_dem = None

"""Stores the most recent PointCloud2 message received"""
def on_pc_update(pc):
    global point_cloud
    point_cloud = pc

"""Converts Point Cloud to Local DEM

Given an array representing the points in the area in front of the robot, this
method creates and returns a local DEM (Digital Elevation Map) representation
of the point cloud. That is, it creates and returns a top-down grid view (as a
2D numpy array), where each cell represents the height at that point. If there
are multiple points mapping to the same cell in the local DEM, the height value
chosen for that cell is chosen according to the given comparison type ("mean",
"median", "min", or "max").
"""
def point_cloud_to_local_dem(pc, comparison_type):
    # Get the comparison function to use when multiple points map to the same
    # cell in the local DEM
    comparison = local_dem_comparison_options[comparison_type]

    local_dem = np.empty((num_rows, num_columns), dtype=object)

    # Get row and column in local DEM array and height for each point in pc
    rows, columns, heights = get_local_dem_info(pc)

    # Create list of heights at each cell in local DEM
    for row, column, height in zip(rows, columns, heights):
        if local_dem[row, column] is None:
            local_dem[row, column] = [height]
        else:
            local_dem[row, column].append(height)

    # For each cell in local DEM, choose a final value for the cell based on
    # the heights of the points that map to that cell. If no points map to
    # that cell, use NaN as the height value.
    for i in range(num_rows):
        for j in range(num_columns):
            if local_dem[i, j] is None:
                local_dem[i, j] = float("nan")
            else:
                local_dem[i, j] = comparison(local_dem[i, j])

    return local_dem.astype(np.float32)

"""Finds the local DEM row, column indexes and heights for a given point cloud

TODO:
- Take into account height of camera and current height of the robot when
  calculating the heights
"""
def get_local_dem_info(pc):
    forward = pc[:,XYZ["FORWARD"]]
    right = pc[:,XYZ["RIGHT"]]
    down = pc[:,XYZ["DOWN"]]

    row_indexes = np.subtract(num_rows-1, np.divide(np.subtract(forward, min_row), resolution).astype(int))
    column_indexes = np.divide(np.subtract(right, min_column), resolution).astype(int)
    heights = np.negative(down)

    return row_indexes, column_indexes, heights

"""Compares global DEM to point clouds to localize the robot

TODO:
- Create global DEM array based on world
- Implement particle filter for localizing
"""
def compare_dem_to_point_cloud(period, local_dem_comparison_type):
    r = rospy.Rate(1./period)
    while not rospy.is_shutdown():
        pc = get_points()
        if pc is not None:
            local_dem = point_cloud_to_local_dem(pc, local_dem_comparison_type)
            # Visualize local DEM (remove later)
            plt.imshow(global_dem);
            plt.colorbar()
            plt.show()
            plt.pause(4.)
        r.sleep()

"""Converts PointCloud2 to numpy array

Reading the most recent PointCloud2 message received, this method creates and
returns a numpy array with all the non-NaN points in the point cloud. If the
point cloud is entirely NaN values, None is returned.
"""
def get_points():
    if point_cloud is None:
        return None

    # Read points directly from point cloud message
    pc = np.frombuffer(point_cloud.data, np.float32)
    # Reshape into array of xyz values
    pc = np.reshape(pc, (-1, 8))[:, :3]
    # Remove nan points
    pc = pc[~np.isnan(pc).any(axis=1)]

    if pc.size > 0:
        return pc
    else:
        return None

"""Returns the angle (in radians) between two 3D rays"""
def angle_between_rays(ray1, ray2):
    dot_product = ray1[0]*ray2[0] + ray1[1]*ray2[1] + ray1[2]*ray2[2]
    magnitude1 = math.sqrt(((ray1[0]**2) + (ray1[1]**2) + (ray1[2]**2)))
    magnitude2 = math.sqrt(((ray2[0]**2) + (ray2[1]**2) + (ray2[2]**2)))
    return math.acos(dot_product / (magnitude1 * magnitude2))

"""Initializes data for creating local DEM

Analyzes a given CameraInfo message to find the minimum and maximum angles the
camera can see. This information, along with the minimum and maximum range the
camera can see and the resolution of the global DEM, is used to determine the
number of rows and columns and the minimum row value and column value so that
a local DEM can be created from a point cloud.
"""
def init_local_dem(camera_info, range_min, range_max):
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

    # Find the range of angles to be covered by the point cloud
    angle_max = angle_between_rays(left_ray, center_ray)
    angle_min = -angle_between_rays(center_ray, right_ray)

    # Find number of rows and columns and minimum row and column values in the
    # local DEM array based on the angles and range the camera can see and the
    # resolution of the global DEM
    global num_rows, num_columns, min_row, min_column
    num_rows = int((range_max - range_min) / resolution) + 1
    num_columns = int((range_max * (np.tan(angle_max) - np.tan(angle_min))) / resolution) + 1
    min_row = range_min
    min_column = range_max * np.tan(angle_min)

# Attempts to get initial elevation from file in dem_data/
def create_array_global_dem(directory):

    # Use list comprehension to get only files in directory as opposed to files and subdirectories
    onlyfiles = [f for f in listdir(directory) if isfile(join(directory, f))]

    # User hasn't put file in dem_data
    if not onlyfiles:
        rospy.logerr("Couldn't read dem data")
    else:
        rospy.loginfo("Reading %s", onlyfiles[0])
        file = open(directory + onlyfiles[0], "r")

        # Reads file line by line, line number starts at 0
        for i, line in enumerate(file):

            # 3rd line of file contains "(rows, cols)"
            if i == 2:

                # Use regex to obtain dimmensions
                dem_size = map(int,re.findall(r'-?(\d+)',line))
                rospy.loginfo(dem_size)

                # File doesn't have dimmensions at line 3
                if not dem_size:
                    rospy.logerr("Couldn't find dem size")
                    break
                else:

                    # Give warning if not square
                    if dem_size[0] != dem_size[1]:
                        rospy.logwarn("Dimmensions are not same value (w != l). Treating as w x w")

                    global global_dem
                    global_dem = np.empty((int(dem_size[0]), int(dem_size[1])), dtype=np.float32)

            elif i > 2:
                global_dem[i - 3] = line.split()



# Find the path to dem_data/
def path_dem():
    rospack = rospkg.RosPack()
    base = rospack.get_path("ezrassor_autonomous_control")
    return base + "/dem_data/"

"""Initializes park ranger"""
def park_ranger(resolution=0.6, local_dem_comparison_type="max", period=5, range_min=0.105, range_max=10.):
    rospy.init_node('park_ranger')
    rospy.loginfo("Park Ranger initialized")
    globals()['resolution'] = resolution
    path = path_dem()
    create_array_global_dem(path)
    camera_info = rospy.wait_for_message("depth/camera_info", CameraInfo)
    init_local_dem(camera_info, range_min, range_max)
    rospy.Subscriber("depth/points", PointCloud2, on_pc_update, queue_size=1)
    compare_dem_to_point_cloud(period, local_dem_comparison_type)

if __name__ == "__main__":
    try:
        park_ranger()
    except:
        pass
