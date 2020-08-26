#!/usr/bin/env python

import rospy
import numpy as np
import nav_functions as nf
from pointcloud_processor import PointCloudProcessor
from gazebo_msgs.msg import LinkStates
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
import cv2
from datetime import datetime
from copy import deepcopy
import matplotlib.pyplot as plt
import rospkg
from os import listdir
from os.path import isfile, join
import re
import filterpy.monte_carlo as mc
from lxml import etree

"""Represents a single particle in the particle filter"""


class Particle:
    def __init__(self, x, y, heading, weight=0):
        self.x = x
        self.y = y
        self.theta = heading
        self.weight = weight


class ParticleFilter:
    def __init__(self, resolution, dem_size):
        self.resolution = resolution
        self.dem_size = dem_size
        self.particles = []
        self.init_particles()

    def add_particle(self, x, y, heading, weight=0):
        self.particles.append(Particle(x, y, heading, weight))

    """Place particle at every cell in global DEM"""

    def init_particles(self):
        weight = 1.0 / (self.dem_size ** 2)
        for y in range(self.dem_size):
            for x in range(self.dem_size):
                self.add_particle(x, y, 0.0, weight)

    """Moves all particles by the given displacement and updates headings"""

    def move_particles(self, x_diff, y_diff, theta):
        for p in self.particles:
            p.x += x_diff
            p.y += -y_diff
            p.theta = theta

        # Remove particles that are now out-of-bounds
        self.particles = [
            p
            for p in self.particles
            if (0 < p.x < self.dem_size) and (0 < p.y < self.dem_size)
        ]

    def resample(self, indexes):
        # Get new particles based on given indexes
        self.particles = [deepcopy(self.particles[i]) for i in indexes]
        # Reset particle weights to 1/N
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
        "min": np.nanmin,
    }

    debug = True

    """Tracks whether the arms are up (camera isn't blocked)"""

    def on_arm_movement(self, data):
        self.arms_up = data

    """Provides robot's z position for simulating altimeteter

    Get the z position (height) of the robot using the simulation state. If
    odometry is not enabled, update x and y positions.
    """

    def sim_state_callback(self, data):
        namespace = rospy.get_namespace()
        namespace = namespace[1:-1] + "::base_link"
        try:
            index = data.name.index(namespace)
        except (AttributeError, ValueError):
            rospy.logdebug("Failed to get index. Skipping...")

        self.position_z = data.pose[index].position.z + self.origin_z

        if not self.real_odometry:
            self.position_x = data.pose[index].position.x
            self.position_y = data.pose[index].position.y
            heading = nf.quaternion_to_yaw(data.pose[index]) * 180 / np.pi
            self.heading = heading % 360

    """Updates x and y position of robot using most recent Odometry message"""

    def odometry_callback(self, data):
        self.position_x = data.pose.pose.position.x + self.start_x
        self.position_y = data.pose.pose.position.y + self.start_y
        heading = nf.quaternion_to_yaw(data.pose.pose) * 180 / np.pi
        self.heading = heading % 360

    """Returns the path to dem_data/"""

    @staticmethod
    def path_dem():
        rospack = rospkg.RosPack()
        base = rospack.get_path("ezrassor_autonomous_control")
        return base + "/dem_data/"

    """Returns the path to worlds/"""

    @staticmethod
    def path_world():
        rospack = rospkg.RosPack()
        base = rospack.get_path("ezrassor_sim_gazebo")
        return base + "/worlds/"

    """Returns the path to weights_results/"""

    @staticmethod
    def weights_results():
        rospack = rospkg.RosPack()
        base = rospack.get_path("ezrassor_autonomous_control")
        return base + "/weights_results/"

    """Returns first heightmap tag from given world file"""

    @staticmethod
    def get_first_heightmap_tag(world_file_string):
        world_root = etree.XML(world_file_string)
        return world_root.find(".//heightmap")

    """Returns the range from the world file"""

    @staticmethod
    def get_world_dem_range(path_world_file):
        file = open(path_world_file, "r")
        first_heightmap_tag = ParkRanger.get_first_heightmap_tag(file.read())
        dem_size_path = first_heightmap_tag.find("size").text
        xyz_size = dem_size_path.split(" ")
        return int(xyz_size[-1])

    """Returns the name of the DEM associated with the world file"""

    @staticmethod
    def get_world_dem_name(path_world_file):
        file = open(path_world_file, "r")
        first_heightmap_tag = ParkRanger.get_first_heightmap_tag(file.read())
        dem_jpg_path = first_heightmap_tag.find("uri").text
        split_by_slash = dem_jpg_path.split("/")
        last_item_slash = split_by_slash[len(split_by_slash) - 1]
        split_by_underscore = last_item_slash.split("_")
        last_item_index = len(split_by_underscore) - 1
        if split_by_underscore[last_item_index] != "converted.jpg":
            return None
        split_by_underscore.pop()
        new_string = ""
        for s in split_by_underscore:
            new_string += s + "_"
        return new_string

    """Returns path of the file associated with world_dem_name in directory"""

    @staticmethod
    def get_match_dem_n_world(world_dem_name, directory):
        # Get only files in directory as opposed to files and subdirectories
        onlyfiles = [
            f for f in listdir(directory) if isfile(join(directory, f))
        ]

        # User hasn't put file in dem_data
        if not onlyfiles:
            rospy.logerr("No files found in dem data")
            return

        for i in onlyfiles:
            split_by_underscore = i.split("_")
            split_by_underscore.pop()
            split_by_underscore.pop()
            new_string = ""
            for s in split_by_underscore:
                new_string += s + "_"
            if world_dem_name == new_string:
                return directory + "/" + i
        return None

    """Creates global DEM from file in dem_data/"""

    def create_array_global_dem(self):
        file = open(self.dem_data_file_path, "r")
        lines = file.readlines()
        dem_size = map(int, re.findall(r"-?(\d+)", lines[2]))

        # File doesn't have dimmensions at line 3
        if not dem_size:
            rospy.logerr("Couldn't find dem size")
            return

        # Give warning if not square
        if dem_size[0] != dem_size[1]:
            rospy.logwarn("Given DEM is not square. Treating as w x w")

        self.dem_size = int(dem_size[0])
        middle = int(dem_size[0] / 2)

        self.global_dem = np.empty(
            (self.dem_size, self.dem_size), dtype=np.float32
        )

        # Read height values into global DEM
        for i, line in enumerate(lines[3:]):
            heights = line.split()
            self.global_dem[i] = heights
            if i == middle:
                self.origin_z = float(heights[middle])

        # "Squishes" the global_dem data to match range found in world file
        old_dem_range = np.ptp(self.global_dem)
        self.global_dem = (self.global_dem * old_dem_range) / self.new_dem_range

    """Initializes data for creating local DEM

    Analyzes a given CameraInfo message to find the minimum and maximum angles
    the camera can see. This information, along with the minimum and maximum
    range the camera can see and the resolution of the global DEM, is used to
    determine the number of rows and columns and the minimum row value and
    column value so that a local DEM can be created from a point cloud.
    """

    def init_local_dem_info(self, range_min, range_max):
        # Find number of rows and columns and minimum row and column values in
        # the local DEM array based on the angles and range the camera can see
        # and the resolution of the global DEM
        self.num_rows = int(range_max - range_min) + 1
        self.num_columns = (
            int(range_max * (np.tan(self.angle_max) - np.tan(self.angle_min)))
            + 1
        )
        self.min_row = range_min
        self.min_column = range_max * np.tan(self.angle_min)

    """Gets the predicted local DEM for a given particle

    Given a particle, this method picks out and returns the portion of the
    global DEM that represents what the robot would see if it were in the state
    described by the particle.
    """

    def get_predicted_local_dem(self, particle):
        # Get angle perpendicular to heading
        angle_perp = (particle.theta + 90) % 360

        predicted_local_dem = np.empty(
            (self.num_rows, self.num_columns), dtype=np.float32
        )

        # To have the predicted local DEM centered around the particle, we need
        # to determine the number of columns on each side of the particle
        num_cols_left = self.num_columns / 2 + 1
        num_cols_right = self.num_columns / 2 + 1
        if self.num_columns % 2 == 0:
            num_cols_right -= 1

        # Get line to the left of the particle
        row_coords = ParkRanger.get_line(
            particle.y, particle.x, num_cols_left, -angle_perp, "left"
        )
        # Fill up columns to the left of the particle
        for i, coord in enumerate(reversed(row_coords)):
            col_coords = ParkRanger.get_line(
                coord[0], coord[1], self.num_rows, particle.theta, "right"
            )
            try:
                predicted_local_dem[:, i] = self.global_dem[
                    col_coords[:, 0], col_coords[:, 1]
                ]
            except IndexError:
                return None

        # Get line to the right of the particle
        row_coords = ParkRanger.get_line(
            particle.y, particle.x, num_cols_right, angle_perp, "right"
        )
        # Fill up columns to the right of the particle
        for i, coord in enumerate(row_coords):
            col_coords = ParkRanger.get_line(
                coord[0], coord[1], self.num_rows, particle.theta, "right"
            )
            try:
                predicted_local_dem[:, num_cols_left - 1 + i] = self.global_dem[
                    col_coords[:, 0], col_coords[:, 1]
                ]
            except IndexError:
                return None

        # Flip predicted local DEM so that it's in the expected orientation
        return np.fliplr(np.flipud(predicted_local_dem))

    """Returns a list of indexes along a 2D array to represent a given angle

    Given a center coordinate for the line, the number of cells we're allowed to
    use to draw the line, and the angle and direction of the line, this method
    calculates and returns the list of indexes that closely approximates the
    line described. This is done using a modified version of Bresenham's line
    algorithm.
    """

    @staticmethod
    def get_line(center_row, center_col, num_cells, angle, direction):
        # Make sure angle is non-negative
        if angle < 0:
            angle += 360

        # Handle cases where tangent returns the same value for different angles
        # and messes up the rest of the calculations
        if direction == "right" and 135 < angle <= 315:
            direction = "left"
            angle = 180 - angle
        if direction == "left" and 45 < angle <= 225:
            direction = "right"
            angle = 180 - angle

        slope = np.tan(angle / 180.0 * np.pi)

        # The below algorithm only works for slope <= 1, so if slope > 1, invert
        # the slope and set a flag so we know to swap the way we treat the axes
        if abs(slope) > 1:
            slope = 1.0 / slope
            line_high = True
        else:
            line_high = False

        # Bresenham's algorithm:
        D = 2 * abs(slope) - 1
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
            D += 2 * abs(slope)
            if line_high:
                row += -1 if direction == "right" else 1
            else:
                col += 1 if direction == "right" else -1

        return np.array(indexes)

    """Produces similarity score between 0 and 2 for the given numpy arrays"""

    @staticmethod
    def histogram_match(a, b):
        indexes = ~np.isnan(a)
        h1 = np.histogram(a[indexes])
        h2 = np.histogram(b[indexes])
        correlation = cv2.compareHist(
            ParkRanger.np_hist_to_cv(h1),
            ParkRanger.np_hist_to_cv(h2),
            cv2.HISTCMP_CORREL,
        )
        return 1 + correlation

    @staticmethod
    def np_hist_to_cv(np_hist):
        counts, bin_edges = np_hist
        return counts.ravel().astype("float32")

    """Converts Point Cloud to Local DEM

    Given an array representing the points in the area in front of the robot,
    this method creates and returns a local DEM (Digital Elevation Map)
    representation of the point cloud. That is, it creates and returns a
    top-down grid view (as a 2D numpy array), where each cell represents the
    height at that point. If there are multiple points mapping to the same cell
    in the local DEM, the height value chosen for that cell is chosen according
    to the given comparison type ("mean", "median", "min", or "max").
    """

    def point_cloud_to_local_dem(self, pc, comparison_type):
        # Get the comparison function to use when multiple points map to the
        # same cell in the local DEM
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

    """Finds local DEM row, column indexes and heights for given point cloud"""

    def get_local_dem_info(self, pc):
        forward = pc[:, PointCloudProcessor.XYZ["FORWARD"]]
        right = pc[:, PointCloudProcessor.XYZ["RIGHT"]]
        down = pc[:, PointCloudProcessor.XYZ["DOWN"]]

        row_indexes = np.subtract(
            self.num_rows - 1, np.subtract(forward, self.min_row)
        ).astype(int)
        column_indexes = np.subtract(right, self.min_column).astype(int)
        heights = np.add(
            np.add(self.position_z, self.camera_height), np.negative(down)
        )

        return row_indexes, column_indexes, heights

    """Returns number of effective particles based on the given weights"""

    @staticmethod
    def neff(norm):
        return 1.0 / np.sum(np.square(norm))

    """Publishes the given position estimate as an Odometry message"""

    def publish_estimate(self, x, y):
        # Initialize odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = rospy.Time.now()
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_link"

        # Set x and y values to x and y estimate
        odom_msg.pose.pose.position.x = float(x)
        odom_msg.pose.pose.position.y = float(y)

        # Set x and y variance to high values
        odom_msg.pose.covariance[0] = 9999.0
        odom_msg.pose.covariance[7] = 9999.0

        self.estimate_pub.publish(odom_msg)

    def __init__(
        self,
        real_odometry,
        world_name,
        resolution,
        local_dem_comparison_type,
        period,
        range_min,
        range_max,
        camera_height,
    ):
        super(ParkRanger, self).__init__("park_ranger")
        self.resolution = resolution
        self.camera_height = camera_height
        self.real_odometry = real_odometry

        # Get spawn x and y coordinates for offsetting odometry messages
        self.start_x = rospy.get_param("autonomous_control/spawn_x_coord")
        self.start_y = rospy.get_param("autonomous_control/spawn_y_coord")

        self.last_x = self.start_x
        self.last_y = self.start_y

        # Get apropriate DEM data file
        path = ParkRanger.path_dem()
        world_name = ParkRanger.path_world() + world_name + ".world"
        world_dem_name = ParkRanger.get_world_dem_name(world_name)
        self.new_dem_range = ParkRanger.get_world_dem_range(world_name)
        self.dem_data_file_path = ParkRanger.get_match_dem_n_world(
            world_dem_name, path
        )
        if not self.dem_data_file_path:
            rospy.logerr("Couldn't find match")
        else:
            # Initialize global DEM
            self.create_array_global_dem()

        # Initialize values used during local DEM processing
        self.init_local_dem_info(range_min, range_max)

        # Subscribe to "altimeter" data, waiting for at least one message
        # moving on
        link_states = rospy.wait_for_message("/gazebo/link_states", LinkStates)
        self.sim_state_callback(link_states)
        rospy.Subscriber(
            "/gazebo/link_states", LinkStates, self.sim_state_callback
        )

        if real_odometry:
            rospy.Subscriber(
                "odometry/filtered", Odometry, self.odometry_callback
            )

        self.arms_up = False
        rospy.Subscriber("arms_up", Bool, self.on_arm_movement)

        self.estimate_pub = rospy.Publisher(
            "park_ranger/odom", Odometry, queue_size=10
        )

        self.run(period, local_dem_comparison_type)

        rospy.loginfo("Park Ranger initialized")

    """Compares global DEM to point clouds to localize the robot"""

    def run(self, period, local_dem_comparison_type):
        r = rospy.Rate(1.0 / period)
        init_flag = True
        self.particle_filter = ParticleFilter(self.resolution, self.dem_size)
        while not rospy.is_shutdown():
            if not self.arms_up:
                continue

            pc = self.get_points()
            if pc is not None:
                local_dem = self.point_cloud_to_local_dem(
                    pc, local_dem_comparison_type
                )

                if not init_flag:
                    if ParkRanger.debug:
                        rospy.logwarn("Prediction step")
                    self.predict_particle_movement()

                    if ParkRanger.debug:
                        rospy.logwarn("Update step")
                    self.likelihood(local_dem)

                    if ParkRanger.debug:
                        rospy.logwarn(
                            "Actual position: {} {}".format(
                                self.position_x, self.position_y
                            )
                        )
                        rospy.logwarn("Estimation step")
                    est_x, est_y = self.estimate()

                    if est_x is not None and est_y is not None:
                        est_x = est_x - (self.dem_size / 2)
                        est_y = (self.dem_size / 2) - est_y
                        if ParkRanger.debug:
                            rospy.logwarn(
                                "Estimated position: {} {}".format(est_x, est_y)
                            )

                        # Get number of unique (in terms of x, y vals) particles
                        num_unique = len(
                            set(
                                [
                                    (p.x, p.y)
                                    for p in self.particle_filter.particles
                                ]
                            )
                        )
                        # If particle filter has nearly converged,
                        # publish estimate
                        if num_unique <= 5:
                            self.publish_estimate(est_x, est_y)

                    # Resample step
                    if ParkRanger.debug:
                        rospy.logwarn("Resample step")
                    self.resample()
                elif init_flag:
                    # Initialization step
                    if ParkRanger.debug:
                        rospy.logwarn("Initialization step")
                    self.likelihood(local_dem)
                    init_flag = False
            r.sleep()

    """Updates weight of each particle by comparing local and predicted DEMs"""

    def likelihood(self, local_dem):
        img_path = self.weights_results()
        weights = np.zeros_like(self.global_dem)

        for p in self.particle_filter.particles:
            predicted_dem = self.get_predicted_local_dem(p)
            if predicted_dem is not None:
                p.weight *= ParkRanger.histogram_match(local_dem, predicted_dem)
                weights[p.y, p.x] = p.weight
            else:
                p.weight = 0.0

        # Save image where each pixel represents a particle (x,y) and weight
        plt.imshow(weights)
        plt.colorbar()
        now = datetime.now()
        current_time = now.strftime("%H:%M:%S")
        plt.savefig(
            "{}{}_{}.png".format(img_path, "weights", current_time),
            bbox_inches="tight",
        )
        plt.clf()

    """Move particles based on movement of robot"""

    def predict_particle_movement(self):
        # Compare movement of robot and adjust particles accordingly
        current_x = self.position_x
        current_y = self.position_y
        current_heading = self.heading

        x_diff = int(current_x - self.last_x)
        y_diff = int(current_y - self.last_y)

        self.particle_filter.move_particles(x_diff, y_diff, current_heading)

        self.last_x = current_x
        self.last_y = current_y

    """Produce estimate for robot's position via weighted sum of particles"""

    def estimate(self):
        sum_x = 0
        sum_y = 0
        sum_weights = sum([p.weight for p in self.particle_filter.particles])
        for p in self.particle_filter.particles:
            if p.weight != 0.0:
                sum_x += p.x * p.weight
                sum_y += p.y * p.weight
        if sum_weights == 0:
            rospy.logwarn("Sum of particle weights is zero")
            return None, None
        return int(sum_x / sum_weights), int(sum_y / sum_weights)

    """Perform resampling if neff value drops below threshold"""

    def resample(self):
        weights = [p.weight for p in self.particle_filter.particles]
        norm = weights / np.linalg.norm(weights)
        if ParkRanger.neff(norm) < (len(norm) / 8):
            indexes = mc.systematic_resample(norm)
            self.particle_filter.resample(indexes)


"""Initializes park ranger"""


def park_ranger(
    real_odometry,
    world_name,
    resolution=0.5,
    local_dem_comparison_type="max",
    period=5,
    range_min=0.105,
    range_max=10.0,
    camera_height=0.34,
):

    """Usage:
    pr = ParkRanger(
        real_odometry,
        world_name,
        resolution,
        local_dem_comparison_type,
        period,
        range_min,
        range_max,
        camera_height,
    )
    """


if __name__ == "__main__":
    try:
        park_ranger()
    except (AttributeError, ValueError, IndexError):
        pass
