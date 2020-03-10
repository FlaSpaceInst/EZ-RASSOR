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


"""Represents a single particle in the particle filter"""
class Particle:
    def __init__(self, row, col, heading):
        self.row = row
        self.col = col
        self.heading = heading

"""Stores the most recent PointCloud2 message received"""
def on_pc_update(pc):
    global point_cloud
    point_cloud = pc

"""Gets the predicted local DEM for a given particle

Given a particle, this method picks out and returns the portion of the global
DEM that represents what the robot would see if it were in the state described
by the particle.
"""
def get_predicted_local_dem(particle):
    # Get angle perpendicular to heading
    angle_perp = (particle.heading + 90) % 360

    predicted_local_dem = np.empty((num_rows, num_columns), np.float32)

    # To have the predicted local DEM centered around the particle, we need to
    # to determine the number of columns on each side of the particle
    num_cols_left = num_columns/2 + 1
    num_cols_right = num_columns/2 + 1
    if num_columns % 2 == 0:
        num_cols_right -= 1

    # Get line to the left of the particle
    row_coords = get_line(particle.row, particle.col, num_cols_left, -angle_perp, 'left')
    # Fill up columns to the left of the particle
    for i, coord in enumerate(reversed(row_coords)):
        col_coords = get_line(coord[0], coord[1], num_rows, particle.heading, 'right')
        try:
            predicted_local_dem[:, i] = global_dem[col_coords[:,0], col_coords[:,1]]
        except IndexError:
            return None

    # Get line to the right of the particle
    row_coords = get_line(particle.row, particle.col, num_cols_right, angle_perp, 'right')
    # Fill up columns to the right of the particle
    for i, coord in enumerate(row_coords):
        col_coords = get_line(coord[0], coord[1], num_rows, particle.heading, 'right')
        try:
            predicted_local_dem[:, num_cols_left-1+i] = global_dem[col_coords[:,0], col_coords[:,1]]
        except IndexError:
            return None

    # Flip predicted local DEM so that it's in the expected orientation
    return np.fliplr(np.flipud(predicted_local_dem))

"""Returns a list of indexes along a 2D array to represent a given angle

Given a center coordinate for the line, the number of cells we're allowed to
use to draw the line, and the angle and direction of the line, this method
calculates and returns the list of indexes that closely approximates the line
described. This is done using a modified version of Bresenham's line algorithm.
"""
def get_line(center_row, center_col, num_cells, angle, direction):
    # Make sure angle is non-negative
    if angle < 0:
        angle += 360

    # Handles cases where tangent returns the same value for different angles
    # and messes up the rest of the calculations
    if direction is 'right' and 135 < angle <= 315:
        direction = 'left'
        angle = 180 - angle
    if direction is 'left' and 45 < angle <= 225:
        direction = 'right'
        angle = 180 - angle

    slope = np.tan(angle / 180. * np.pi)

    # The below algorithm only works for slope <= 1, so if slope > 1, invert
    # the slope and set a flag so we know to swap the way we treat the axes
    if abs(slope) > 1:
        slope = 1. / slope
        line_high = True
    else:
        line_high = False

    # Bresenham's algorithm:
    D = 2*abs(slope) - 1
    row = center_row
    col = center_col
    indexes = []
    for i in range(num_cells):
        indexes.append([row, col])
        if D > 0:
            if line_high:
                col += 1 if slope > 0 else -1
            else:
                row += -1 if slope > 0 else 1
            D -= 2
        D += 2*abs(slope)
        if line_high:
            row += -1 if direction is 'right' else 1
        else:
            col += 1 if direction is 'right' else -1

    return np.array(indexes)

"""Returns the sum of absolute differences (SAD) of two arrays"""
def sad(a, b):
    return np.nansum(np.absolute(np.subtract(a, b)))

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
- Implement particle filter for localizing
"""
def compare_dem_to_point_cloud(period, local_dem_comparison_type):
    r = rospy.Rate(1./period)
    while not rospy.is_shutdown():
        pc = get_points()
        if pc is not None:
            local_dem = point_cloud_to_local_dem(pc, local_dem_comparison_type)
            particle = Particle(370, 160, 36)
            predicted_dem = get_predicted_local_dem(particle)
            if predicted_dem is not None:
                score = sad(local_dem, predicted_dem)
                rospy.loginfo("Predicted local DEM score: {}".format(score))
                # Visualize DEM (remove later)
                plt.imshow(predicted_dem);
                plt.colorbar()
                plt.show(block=False)
                plt.pause(4.)
                plt.close()
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
