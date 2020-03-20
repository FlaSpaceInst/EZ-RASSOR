#!/usr/bin/env python

import rospy
from sensor_msgs.msg import PointCloud2, CameraInfo
import numpy as np
import math
import image_geometry
import nav_functions as nf

from sklearn.preprocessing import normalize

# **Remove later (used for visualization of local DEM only)
import matplotlib.pyplot as plt

import rospkg
from os import listdir
from os.path import isfile, join
import re

import random
from scipy.stats import truncnorm
import gvar as gv
# for residual_resample
import filterpy.monte_carlo as mc

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
    def __init__(self, id, row, col, heading, weight=0, std_x=0, std_y=0, std_theta=0):
        self.id = id
        self.x = row
        self.y = col
        self.theta = heading
        self.weight = weight
        # Error function
        if std_x != 0 or std_y != 0 or std_theta != 0:
            self.distra_x = gv.gvar(row, std_x)
            self.distra_y = gv.gvar(col, std_y)
            self.distra_theta = gv.gvar(heading, std_theta)
        else:
            self.distra_x = None
            self.distra_y = None
            self.distra_theta = None

    def set_distras(self, std_x, std_y, std_theta):
        self.distra_x = gv.gvar(int(self.x), std_x)
        self.distra_y = gv.gvar(int(self.y), std_y)
        self.distra_theta = gv.gvar(float(self.theta), std_theta)

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
    angle_perp = (particle.theta + 90) % 360

    predicted_local_dem = np.empty((num_rows, num_columns), np.float32)

    # To have the predicted local DEM centered around the particle, we need to
    # to determine the number of columns on each side of the particle
    num_cols_left = num_columns/2 + 1
    num_cols_right = num_columns/2 + 1
    if num_columns % 2 == 0:
        num_cols_right -= 1

    # Get line to the left of the particle
    row_coords = get_line(particle.x, particle.y, num_cols_left, -angle_perp, 'left')
    # Fill up columns to the left of the particle
    for i, coord in enumerate(reversed(row_coords)):
        col_coords = get_line(coord[0], coord[1], num_rows, particle.theta, 'right')
        try:
            predicted_local_dem[:, i] = global_dem[col_coords[:,0], col_coords[:,1]]
        except IndexError:
            return None

    # Get line to the right of the particle
    row_coords = get_line(particle.x, particle.y, num_cols_right, angle_perp, 'right')
    # Fill up columns to the right of the particle
    for i, coord in enumerate(row_coords):
        col_coords = get_line(coord[0], coord[1], num_rows, particle.theta, 'right')
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

# Auxilary functions
def neff(weights):
    norm = [float(i)/sum(weights) for i in weights]
    return 1.0 / np.sum(np.square(norm))

def get_truncated_normal(mean=0, sd=1, low=0, upp=10):
    return truncnorm(
        (low - mean) / sd, (upp - mean) / sd, loc=mean, scale=sd)

# Buggy when updating ids, possibly just ditch index id and use unique ids
# Helper function for sampling step, originally used for psuedo initialzation, need to clean this up
def particles_w_distra(particles, N, dem_size, index, diff_theta, samp_part, samp_std_x, samp_std_y, samp_std_theta):
    mean_x = int(dem_size / 2)
    mean_y = mean_x
    mean_theta = 74 / 2
    std_x = int(dem_size / 2)
    std_y = std_x
    std_theta = 74 / 2
    bound_x = dem_size
    bound_y = bound_x
    bound_theta = 359
    low_x = 0
    low_y = low_x
    low_theta = 0
    start = 0
    end = N
    if index == -1:
        index = 0
    elif(diff_theta != -1 and samp_part != None and index != -1):
        for i in range(end, len(particles)):
            particles[i].id = i + N
        start = 1 + index
        end = N + 1 + index
        bound_x = samp_part.x + end
        bound_y = samp_part.y + end
        bound_theta = samp_part.theta + diff_theta
        low_x = abs(samp_part.x - N)
        low_y = abs(samp_part.y - N)
        low_theta = samp_part.theta - diff_theta
        mean_x = samp_part.x
        mean_y = samp_part.y
        mean_theta = samp_part.theta
        std_x = samp_std_x
        std_y = samp_std_y
        std_theta = samp_std_theta
    x_func = get_truncated_normal(mean_x, std_x, low_x, bound_x)
    y_func = get_truncated_normal(mean_y, std_y, low_y, bound_y)
    theta_func = get_truncated_normal(mean_theta, std_theta, low_theta, bound_theta)
    i = start
    while i < end:
        x = int(x_func.rvs())
        y = int(y_func.rvs())
        theta = theta_func.rvs()
        particles.insert(i, Particle(i, x, y, theta, 1.0 / N, std_x, std_y, std_theta))
        i = i + 1

def sampling(particles, num_particles, threshold_point, threshold_theta, dem_size, samp_std_x, samp_std_y, samp_std_theta):
    std_coord = int(dem_size / 2)
    std_theta = (74 / 2)
    rand_func = get_truncated_normal(int(num_particles) / 2, 10, 1, num_particles)
    rand_id = int(rand_func.rvs())
    x_comp = abs(particles[rand_id].distra_x.sdev - particles[rand_id].x)
    y_comp = abs(particles[rand_id].distra_y.sdev - particles[rand_id].y)
    theta_comp = abs(particles[rand_id].distra_theta.sdev - particles[rand_id].theta)
    if x_comp > threshold_point or y_comp > threshold_point or theta > threshold_theta:
        particles_w_distra(particles, 25, dem_size, rand_id, theta_comp, particles[rand_id], samp_std_x, samp_std_y, samp_std_theta)
        for i in range(rand_id + 1, rand_id + 26):
            particles[i].distra_x = gv.gvar(particles[i].distra_x.mean, std_coord)
            particles[i].distra_y = gv.gvar(particles[i].distra_y.mean, std_coord)
            particles[i].distra_theta = gv.gvar(particles[i].distra_theta.mean, std_theta)
        particles[rand_id].distra_x = gv.gvar(particles[rand_id].distra_x.mean, std_coord)
        particles[rand_id].distra_y = gv.gvar(particles[rand_id].distra_y.mean, std_coord)
        particles[rand_id].distra_theta = gv.gvar(particles[rand_id].distra_theta.mean, std_theta)

def resample_from_index(particles, weights, indexes):
    particles[:] = particles[indexes]
    rospy.logwarn("Before: {}".format(weights))
    weights[:] = weights[indexes]
    rospy.logwarn("After: {}".format(weights))
    weights.fill(1.0 / len(weights))

def resample(particles, N):
    weights = []
    for i in particles:
        weights.append(i.weight)
    norm = weights / np.linalg.norm(weights)
    if neff(norm) < N / 2:
        rospy.logwarn("Resample")
        indexes = mc.systematic_resample(norm)
        rospy.logwarn("indexes {}".format(indexes))
        #resample_from_index(particles, weights, indexes)


def init_check(particles, N):
    rand_theta = get_truncated_normal(int(74/2),5,0,359)
    for i in range(0, N):
        for j in range(0, N):
            rospy.logwarn("Gen at: {}, {}".format(i, j))
            particles.append(Particle(-1, i, j, float(rand_theta.rvs()), 1.0 / N, -1, -1, -1))

def likelihood(particles, local_dem):
    for p in particles:
        #particle = Particle(370, 160, 36)
        predicted_dem = get_predicted_local_dem(p)
        if predicted_dem is not None:
            score = sad(local_dem, predicted_dem)
            p.weight = p.weight * score
            rospy.logwarn("Predicted local DEM score: {}".format(p.weight))
        # Visualize DEM (remove later)
        #plt.imshow(predicted_dem);
        #plt.colorbar()
        #plt.show(block=False)
        #plt.pause(4.)
        #plt.close()
        #plt.plot([p.x], [p.y], marker='o', markersize=3, color="red")
    #plt.show()
    #plt.pause(4.)
    #plt.close()

def place_high_like_parts(particles, N):
    # Sort particles by weight in descending order
    place = sorted(particles, key=lambda x: x.weight, reverse=True)
    particles = []

    # Get and "place" the N highest weighted particles and reset weights and gaussian
    for i in range(0, N):
        p = place.pop(0)
        p.weight = 1.0 / N
        p.set_distras(N / 2, N / 2, int(74 / 2))
        particles.append(p)
    return particles

def get_odom_covar_n_diplace(prev_odom_msg, curr_odom_msg):
    covar = curr_odom_msg.pose.covariance
    # Absolute
    heading = nf.quaternion_to_yaw(curr_odom_msg.pose.orientation - prev_odom_msg.pose.orientation)
    x_diff = (curr_odom_msg.pose.position.x - prev_odom_msg.pose.position.x)
    y_diff = (curr_odom_msg.pose.position.y - prev_odom_msg.pose.position.y)
    return covar, heading, x_diff, y_diff, theta_diff

def predict_particle_movement(particle, dem_bound):
    # Compare odometry messages and adjust particle physically and probability
    get_odom_covar_n_diplace(prev_odom, curr_odom)
    particle.x = particle.x + x_diff
    particle.y = particle.y + y_diff
    particle.theta = (particle.theta + theta_diff) % 360
    if particle.x > dem_bound or particle.y > dem_bound:
        rospy.logwarn("Out of DEM bounds at: {}, {}".format(particle.x, particle.y))
    # TODO: adjust gaussians

# Background:
# The main steps of "simple" particle filter are:
    # Predict: using odometry to "predict" the position of a particle after one pass
    # Update: compare data from, usually using a laser scan, to a "predicted" laser scan for each
        # particle, and then they are scored. This score is then used to weight the particle
    # Resample: looking at the weights, use neff  or some other function to find an overall score
        # for the set of particles. If below a threshold, they usually just replace the particles with
        # a fresh batch, probably with some of the previous weighted values. This is done to get rid of incorrect particles
# There are 2 additional steps the paper adds
    # Initialization: obvi you need initialzation but this one basically supposed if there are particles at every
    # position and then only uses the highest likelihood ones
    # Sampling: this is generating more particles to account for uncertainty
# Other unconventional stuff:
    # Instead of keeping a global standard deviations for like x, y, theta, they add an error function to each particle.
    # With global standard deviations, they would normally use it to essential inject noise into calculations. With an error
    # function, it seems like they are putting an error radius around a particle.

# The Paper's Particle Filter Steps

# 1 Initialize
    # a) use likelihood (local to global compare) to initially score positions (essential evaluate as if particle at every position in global dem)
    # b) place the highest likelihood particles
    # c) for each particle, give it a weight and an error distribution

# 2 Predict (aka predict movement of particle)
    # a) for each particle, use motion vector to predict new position
        # - Use odometry message(s) (pose don't use twist, i'm avoiding physics math) to predict a position + orientation
        # - Thinking about keeping track of previous odometry message and compare current one to
        #   predict position but subscribers are async so to do this, need global variables
        # - Could also try to get the world_state pose published but might need to completely change the architecture for that
        #   cuz of how ros nodes do progress guarantee or whatever it's called in concurrency
    # b) update gaussian distribution (error distribution function) by replacing it
    # with a new distribution but using the sum of the current one's variance and one found from odometry
        # - variance = standard deviation^2 -> just sum the standard deviations, only need more than one odom object
        # - could use the covariance matrix possibly to find the standard deviation (see gvar library for python)

# 3 Sample
    # a) for each particle or some particle? wasn't sure which one but in this case, i do it for a random particle
    # b) Compare x, y, and z, to their error distribution's standard deviation and see how much off
    # c) if the either of the differences is above the threshold, generate particles around it
    # d) update the standard deviation for the particle from a) and the newly generate particles to dem_size/2 or 74/2 (74 is intel fov yaw)

    # My uncertainty on this implementation comes from this:
        # "Given a particle i, its gaussian distribution standard deviation are compared with the coordinate resolution and with the angular resolution"
        # - I'm unsure about the "resolution" values, not sure if it's like the size of the local dem but in this case, i went with using the coordinates of
        # the particle since the goal of the function is to gen particles around some particle i to account for error

# 4 Update
    # for each particle
    # a) score the particle (compare local_dem to predicted_dem)
    # b) update weight of particle

# 5 Resample
    # a) check the equation is below the threshold
    # b) if true, resample particles with probabilities given by there weights ?????
    # c) weights are then reinitialized to 1/N


"""Compares global DEM to point clouds to localize the robot

TODO:
- Rename this to particle filter
"""

def compare_dem_to_point_cloud(period, local_dem_comparison_type, particles, N):
    r = rospy.Rate(1./period)
    init_flag = True
    while not rospy.is_shutdown():
        pc = get_points()
        if pc is not None:
            local_dem = point_cloud_to_local_dem(pc, local_dem_comparison_type)
            if init_flag == False:
                # 3 Sampling
                rospy.logwarn("Sampling")
                sampling(particles, len(particles), 15, 25, 513, 1, 1, 5)
                # 4 Update
                rospy.logwarn("Update")
                likelihood(particles, local_dem)
                # 5 Resample
                rospy.logwarn("Resample")
                resample(particles, len(particles))
            else:
                # 1 Initialization
                rospy.logwarn("Initialize")
                # According to the paper, this would generate a particle at every possible dem
                # For the sake of not taking forever, just set this to 50
                init_check(particles, 50)
                likelihood(particles, local_dem)
                # rather than a NxN operation like previous two right above,
                # for the sorting, lambda function is used so fast af and N is used to
                # pop the list of N highest scored particles
                particles = place_high_like_parts(particles, N)
                init_flag = False
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
    num_particles = 50
    particles = []
    rospy.init_node('park_ranger')
    rospy.loginfo("Park Ranger initialized")
    globals()['resolution'] = resolution
    path = path_dem()
    create_array_global_dem(path)
    camera_info = rospy.wait_for_message("depth/camera_info", CameraInfo)
    init_local_dem(camera_info, range_min, range_max)
    rospy.Subscriber("depth/points", PointCloud2, on_pc_update, queue_size=1)
    #particles_w_distra(particles, num_particles, 513, -1, -1, None, -1, -1, -1)
    #for i in range(0, 10):
    compare_dem_to_point_cloud(period, local_dem_comparison_type, particles, 513)
    #new_num_part = len(particles)
    #resample(particles, new_num_part)

if __name__ == "__main__":
    try:
        park_ranger()
    except:
        pass
