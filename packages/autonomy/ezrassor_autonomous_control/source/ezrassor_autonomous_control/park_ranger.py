#!/usr/bin/env python

import rospy
import numpy as np
import nav_functions as nf
from pointcloud_processor import PointCloudProcessor
from gazebo_msgs.msg import LinkStates
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool

from sklearn.preprocessing import normalize

# **Remove later (used for visualization of local DEM only)
import matplotlib
#matplotlib.use('Agg')
import matplotlib.pyplot as plt

import rospkg
from os import listdir
from os.path import isfile, join
import re

import random
from scipy.stats import truncnorm
# for residual_resample
import filterpy.monte_carlo as mc

"""
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
"""

"""Represents a single particle in the particle filter"""
class Particle:
    def __init__(self, row, col, heading, weight=0, std_x=0, std_y=0):
        self.x = row
        self.y = col
        self.theta = heading
        self.weight = weight
        self.std_x = std_x
        self.std_y = std_y

class ParticleFilter:
    def __init__(self, resolution, dem_min_x, dem_max_x, dem_min_y, dem_max_y):
        self.resolution = resolution
        self.dem_min_x = dem_min_x
        self.dem_max_x = dem_max_x
        self.dem_min_y = dem_min_y
        self.dem_max_y = dem_max_y
        self.particles = []

    def add_particle(self, row, col, heading, weight=0, std_x=0, std_y=0):
        self.particles.append(Particle(row, col, heading, weight, std_x, std_y))

    def move_particles(self, x_diff, y_diff, theta, std_x, std_y):
        for p in self.particles:
            p.x += x_diff
            p.y += y_diff
            p.theta = theta
            p.std_x += std_x
            p.std_y += std_y

        self.particles = [p for p in self.particles if (self.dem_min_x < p.x < self.dem_max_x) and (self.dem_min_y < p.y < self.dem_max_y)]

    def sample(self):
        for p in self.particles:
            x_comp = int(p.std_x - self.resolution)
            if x_comp > 0:
                for i in range(x_comp):
                    p.std_x = self.resolution / 2.0
                    self.particles.append(Particle(p.x-i, p.y, p.theta, p.weight, p.std_x, p.std_y))
                    self.particles.append(Particle(p.x+i, p.y, p.theta, p.weight, p.std_x, p.std_y))

            y_comp = int(p.std_y - self.resolution)
            if y_comp > 0:
                for i in range(y_comp):
                    p.std_y = self.resolution / 2.0
                    self.particles.append(Particle(p.x, p.y-i, p.theta, p.weight, p.std_x, p.std_y))
                    self.particles.append(Particle(p.x, p.y+i, p.theta, p.weight, p.std_x, p.std_y))

    def resample(self, indexes):
        indexes = np.unique(indexes)
        self.particles = [self.particles[i] for i in indexes]
        num_particles = len(self.particles)
        for p in self.particles:
            p.weight = 1.0 / num_particles

class ParkRanger(PointCloudProcessor):
    # Used to determine what value to use for a cell in the local DEM when
    # multiple points fall into that cell
    local_dem_comparison_options = {
        "mean": np.nanmean,
        "median": np.nanmedian,
        "max": np.nanmax,
        "min": np.nanmin
    }

    debug = True

    def on_arm_movement(self, data):
        self.arms_up = data

    """Stores most recent Z position of robot"""
    def sim_state_z_position_callback(self, data):
        namespace = rospy.get_namespace()
        namespace = namespace[1:-1]+"::base_link"
        try:
            index = data.name.index(namespace)
        except:
            rospy.logdebug("Failed to get index. Skipping...")

        self.position_z = data.pose[index].position.z + self.origin_z

    def odometry_callback(self, data):
        self.covar = data.pose.covariance
        self.position_x = data.pose.pose.position.x + self.start_x
        self.position_y = data.pose.pose.position.y + self.start_y

        heading = nf.quaternion_to_yaw(data.pose.pose) * 180/np.pi

        self.heading = heading % 360

    """Returns the path to dem_data/"""
    @staticmethod
    def path_dem():
        rospack = rospkg.RosPack()
        base = rospack.get_path("ezrassor_autonomous_control")
        return base + "/dem_data/"

    """Creates global DEM from file in dem_data/"""
    def create_array_global_dem(self, directory):
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

                        self.dem_size = dem_size[0]

                        middle = int(dem_size[0] / 2)

                        self.global_dem = np.empty((int(dem_size[0]), int(dem_size[1])), dtype=np.float32)

                elif i > 2:
                    self.global_dem[i - 3] = line.split()

                    if (i-3) == middle:
                        # Split by white space, then find the middle value on the level
                        temp = line.split()
                        self.origin_z = float(temp[middle])

    """Initializes data for creating local DEM

    Analyzes a given CameraInfo message to find the minimum and maximum angles the
    camera can see. This information, along with the minimum and maximum range the
    camera can see and the resolution of the global DEM, is used to determine the
    number of rows and columns and the minimum row value and column value so that
    a local DEM can be created from a point cloud.
    """
    def init_local_dem_info(self, range_min, range_max):
        # Find number of rows and columns and minimum row and column values in the
        # local DEM array based on the angles and range the camera can see and the
        # resolution of the global DEM
        self.num_rows = int((range_max - range_min) / self.resolution) + 1
        self.num_columns = int((range_max * (np.tan(self.angle_max) - np.tan(self.angle_min))) / self.resolution) + 1
        self.min_row = range_min
        self.min_column = range_max * np.tan(self.angle_min)

    """Gets the predicted local DEM for a given particle

    Given a particle, this method picks out and returns the portion of the global
    DEM that represents what the robot would see if it were in the state described
    by the particle.
    """
    def get_predicted_local_dem(self, particle):
        # Get angle perpendicular to heading
        angle_perp = (particle.theta + 90) % 360

        predicted_local_dem = np.empty((self.num_rows, self.num_columns), np.float32)

        # To have the predicted local DEM centered around the particle, we need to
        # to determine the number of columns on each side of the particle
        num_cols_left = self.num_columns/2 + 1
        num_cols_right = self.num_columns/2 + 1
        if self.num_columns % 2 == 0:
            num_cols_right -= 1

        # Get line to the left of the particle
        row_coords = ParkRanger.get_line(particle.x, particle.y, num_cols_left, -angle_perp, 'left')
        # Fill up columns to the left of the particle
        for i, coord in enumerate(reversed(row_coords)):
            col_coords = ParkRanger.get_line(coord[0], coord[1], self.num_rows, particle.theta, 'right')
            try:
                predicted_local_dem[:, i] = self.global_dem[col_coords[:,0], col_coords[:,1]]
            except IndexError:
                return None

        # Get line to the right of the particle
        row_coords = ParkRanger.get_line(particle.x, particle.y, num_cols_right, angle_perp, 'right')
        # Fill up columns to the right of the particle
        for i, coord in enumerate(row_coords):
            col_coords = ParkRanger.get_line(coord[0], coord[1], self.num_rows, particle.theta, 'right')
            try:
                predicted_local_dem[:, num_cols_left-1+i] = self.global_dem[col_coords[:,0], col_coords[:,1]]
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
    @staticmethod
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
    @staticmethod
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
    def point_cloud_to_local_dem(self, pc, comparison_type):
        # Get the comparison function to use when multiple points map to the same
        # cell in the local DEM
        comparison = ParkRanger.local_dem_comparison_options[comparison_type]

        local_dem = np.empty((self.num_rows, self.num_columns), dtype=object)

        # Get row and column in local DEM array and height for each point in pc
        rows, columns, heights = self.get_local_dem_info(pc)

        # Create list of heights at each cell in local DEM
        for row, column, height in zip(rows, columns, heights):
            if local_dem[row, column] is None:
                local_dem[row, column] = [height]
            else:
                local_dem[row, column].append(height)

        # For each cell in local DEM, choose a final value for the cell based on
        # the heights of the points that map to that cell. If no points map to
        # that cell, use NaN as the height value.
        for i in range(self.num_rows):
            for j in range(self.num_columns):
                if local_dem[i, j] is None:
                    local_dem[i, j] = float("nan")
                else:
                    local_dem[i, j] = comparison(local_dem[i, j])

        return local_dem.astype(np.float32)

    """Finds the local DEM row, column indexes and heights for a given point cloud"""
    def get_local_dem_info(self, pc):
        forward = pc[:,PointCloudProcessor.XYZ["FORWARD"]]
        right = pc[:,PointCloudProcessor.XYZ["RIGHT"]]
        down = pc[:,PointCloudProcessor.XYZ["DOWN"]]

        row_indexes = np.subtract(self.num_rows-1, np.divide(np.subtract(forward, self.min_row), self.resolution).astype(int))
        column_indexes = np.divide(np.subtract(right, self.min_column), self.resolution).astype(int)
        heights = np.add(np.add(self.position_z, self.camera_height), down)

        return row_indexes, column_indexes, heights

    # Auxilary functions
    @staticmethod
    def neff(norm):
        return 1.0 / np.sum(np.square(norm))

    @staticmethod
    def get_truncated_normal(mean=0, sd=1, low=0, upp=10):
        return truncnorm(
            (low - mean) / sd, (upp - mean) / sd, loc=mean, scale=sd)

    def plot_points(self):
        plt.ylim(0, self.particle_filter.dem_max_y)
        plt.xlim(0, self.particle_filter.dem_max_x)
        for p in self.particle_filter.particles:
            plt.plot([p.x], [p.y], marker='o', markersize=3, color="red")
        plt.show(block=False)
        plt.pause(3)
        plt.close()

    def __init__(self, resolution, local_dem_comparison_type, period,
                 range_min, range_max, camera_height):
        super(ParkRanger, self).__init__('park_ranger')
        self.resolution = resolution
        self.camera_height = camera_height

        # Get spawn x and y coordinates for offsetting odometry messages
        self.start_x = rospy.get_param('autonomous_control/spawn_x_coord')
        self.start_y = rospy.get_param('autonomous_control/spawn_y_coord')

        self.last_x = self.start_x
        self.last_y = self.start_y

        self.num_initial_particles = 50
        self.max_particles = 1000

        path = ParkRanger.path_dem()

        self.create_array_global_dem(path)
        self.init_local_dem_info(range_min, range_max)

        # Subscribe to "altimeter" data, waiting for at least one message
        # moving on
        link_states = rospy.wait_for_message("/gazebo/link_states", LinkStates)
        self.sim_state_z_position_callback(link_states)
        rospy.Subscriber('/gazebo/link_states', LinkStates,
                         self.sim_state_z_position_callback)

        rospy.Subscriber('odometry/filtered', Odometry, self.odometry_callback)

        self.arms_up = False
        rospy.Subscriber('arms_up', Bool, self.on_arm_movement)

        self.run(period, local_dem_comparison_type)

        rospy.loginfo("Park Ranger initialized")

    """Compares global DEM to point clouds to localize the robot"""
    def run(self, period, local_dem_comparison_type):
        r = rospy.Rate(1./period)
        init_flag = True
        self.particle_filter = ParticleFilter(self.resolution, 0, self.dem_size, 0, self.dem_size)
        while not rospy.is_shutdown():
            if not self.arms_up:
                continue

            pc = self.get_points()
            if pc is not None:
                local_dem = self.point_cloud_to_local_dem(pc, local_dem_comparison_type)
                if not init_flag:
                    # 2 Prediction
                    if ParkRanger.debug:
                        rospy.logwarn("Prediction")
                    self.predict_particle_movement()
                    # 3 Sampling
                    if ParkRanger.debug:
                        rospy.logwarn("Sampling")
                    self.particle_filter.sample()
                    self.place_high_like_parts(self.max_particles)
                    # 4 Update
                    if ParkRanger.debug:
                        rospy.logwarn("Update")
                    self.likelihood(local_dem)
                    self.plot_points()
                    # estimate
                    if ParkRanger.debug:
                        rospy.logwarn("Estimate")
                    est_row, est_col = self.estimate() # TODO: convert to x, y
                    if ParkRanger.debug:
                        rospy.logwarn("Current Estimate {} {}".format(est_row, est_col))
                    # 5 Resample
                    if ParkRanger.debug:
                        rospy.logwarn("Resample")
                    self.resample()
                else:
                    # 1 Initialization
                    if ParkRanger.debug:
                        rospy.logwarn("Initialize")
                    # According to the paper, this would generate a particle at every possible dem
                    # For the sake of not taking forever, just set this to 50
                    self.init_particles(500)
                    self.likelihood(local_dem)
                    # rather than a NxN operation like previous two right above,
                    # for the sorting, lambda function is used so fast af and N is used to
                    # pop the list of N highest scored particles
                    self.place_high_like_parts(self.num_initial_particles)
                    init_flag = False
            r.sleep()

    """TODO: Initialize start_row and start_col from start_x and start_y"""
    def init_particles(self, num_particles):
        x_coords = np.random.randint(self.dem_size, size=num_particles)
        y_coords = np.random.randint(self.dem_size, size=num_particles)
        weight = 1.0 / num_particles

        for x, y in zip(x_coords, y_coords):
            self.particle_filter.add_particle(x, y, 0.0, weight)

        # x_func = ParkRanger.get_truncated_normal(self.dem_size/2.0, self.resolution, 0, self.dem_size)
        # y_func = ParkRanger.get_truncated_normal(self.dem_size/2.0, self.resolution, 0, self.dem_size)
        # weight = 1.0 / num_particles
        # for i in range(num_particles):
        #     x = int(x_func.rvs())
        #     y = int(y_func.rvs())
        #     self.particle_filter.add_particle(x, y, 0.0, weight)

    def likelihood(self, local_dem):
        num_particles = len(self.particle_filter.particles)
        for p in self.particle_filter.particles:
            predicted_dem = self.get_predicted_local_dem(p)
            if predicted_dem is not None:
                score = ParkRanger.sad(local_dem, predicted_dem)
                p.weight = p.weight * (score / num_particles)
                if ParkRanger.debug:
                    rospy.logwarn("Predicted local DEM score: {}".format(score))
            else:
                if ParkRanger.debug:
                    rospy.logwarn("Invalid predicted local DEM")
                p.weight = -1

        self.particle_filter.particles = [p for p in self.particle_filter.particles if p.weight >= 0]

    def place_high_like_parts(self, num_particles):
        if len(self.particle_filter.particles) <= num_particles:
            return

        # Sort particles by weight in descending order
        place = sorted(self.particle_filter.particles, key=lambda x: x.weight, reverse=False)
        particles = []

        # Get and "place" the N highest weighted particles and reset weights and gaussian
        for i in range(num_particles):
            p = place.pop(0)
            p.weight = 1.0 / num_particles
            p.std_x = self.resolution / 2
            p.std_y = self.resolution / 2
            particles.append(p)

        self.particle_filter.particles = particles

    def predict_particle_movement(self):
        # Compare odometry messages and adjust particle physically and probability
        covar = self.covar
        current_x = self.position_x
        current_y = self.position_y
        current_heading = self.heading
        col_diff = int((current_x - self.last_x) / self.resolution)
        row_diff = int((current_y - self.last_y) / self.resolution)

        std_col = np.sqrt(covar[0])
        std_row = np.sqrt(covar[7])

        self.particle_filter.move_particles(row_diff, col_diff, current_heading, std_row, std_col)

        self.last_x = current_x
        self.last_y = current_y

    def estimate(self):
        sum_x = 0
        sum_y = 0
        num_particles = len(self.particle_filter.particles)
        sum_weights = sum([p.weight for p in self.particle_filter.particles])
        for p in self.particle_filter.particles:
            #rospy.logwarn("{}".format(p.weight))
            sum_x += (p.x * p.weight)
            sum_y += (p.y * p.weight)
        return int(sum_x / sum_weights), int(sum_y / sum_weights)

    def resample(self):
        rospy.logwarn("num particles: {}".format(len(self.particle_filter.particles)))
        weights = [p.weight for p in self.particle_filter.particles]
        norm = np.divide(weights, np.sum(weights))
        if ParkRanger.neff(norm) <  self.num_initial_particles / 2:
            indexes = mc.systematic_resample(norm)
            self.particle_filter.resample(indexes)

"""Initializes park ranger"""
def park_ranger(resolution=0.5, local_dem_comparison_type="max", period=5,
                range_min=0.105, range_max=10., camera_height=0.08):
    pr = ParkRanger(resolution, local_dem_comparison_type, period, range_min,
                    range_max, camera_height)

if __name__ == "__main__":
    try:
        park_ranger()
    except:
        pass
