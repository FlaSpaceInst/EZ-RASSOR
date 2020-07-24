#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
import numpy as np
from pointcloud_processor import PointCloudProcessor


class ObstacleDetector(PointCloudProcessor):
    def __init__(
        self,
        max_angle,
        max_obstacle_dist,
        min_hole_diameter,
        scan_time,
        range_min,
        range_max,
    ):
        super(ObstacleDetector, self).__init__("obstacle_detection")

        self.max_slope = np.tan(max_angle * np.pi / 180.0)
        self.max_obstacle_dist = max_obstacle_dist
        self.min_hole_diameter = min_hole_diameter
        self.scan_time = scan_time
        self.range_min = range_min
        self.range_max = range_max

        self.hike_pub = rospy.Publisher(
            "obstacle_detection/hike", LaserScan, queue_size=10
        )
        self.slope_pub = rospy.Publisher(
            "obstacle_detection/slope", LaserScan, queue_size=10
        )
        self.combined_pub = rospy.Publisher(
            "obstacle_detection/combined", LaserScan, queue_size=10
        )

        rospy.loginfo("Obstacle Detection initialized.")

    def init_pc_info(self, camera_info):
        super(ObstacleDetector, self).init_pc_info(camera_info)

        # Get LaserScan-specific info from camera_info message
        self.ranges_size = camera_info.width
        self.frame_id = camera_info.header.frame_id
        self.angle_increment = (self.angle_max - self.angle_min) / (
            self.ranges_size - 1
        )

    """Creates and returns a LaserScan object based on the given ranges list."""

    def create_laser_scan(self, ranges):
        scan = LaserScan()
        scan.header.stamp = rospy.Time.now()
        scan.header.frame_id = self.frame_id
        scan.angle_min = self.angle_min
        scan.angle_max = self.angle_max
        scan.angle_increment = self.angle_increment
        scan.time_increment = 0.0
        scan.scan_time = self.scan_time
        scan.range_min = self.range_min
        scan.range_max = self.range_max
        scan.ranges = ranges
        scan.intensities = []
        return scan

    def on_pc_update(self, pc):
        super(ObstacleDetector, self).on_pc_update(pc)
        self.point_cloud_to_laser_scan()

    """ Converts PointCloud2 to LaserScan

    Given a PointCloud2 message representing the area in front of the robot,
    this method calculates the slope and the gap in distance (hike) between
    consecutive points in a given direction. These slope and hike values are
    compared to thresholds to determine the distance to the closest obstacle in
    each direction. Slope is used to detect above-ground (positive) obstacles
    and hike is used to detect holes (negative obstacles).
    """

    def point_cloud_to_laser_scan(self):
        # Initial LaserScans assume infinite travel in every direction
        hike_ranges = [float("nan")] * self.ranges_size
        slope_ranges = [float("nan")] * self.ranges_size
        min_ranges = [float("nan")] * self.ranges_size

        # Populate the point cloud
        pc = self.get_points()

        # Perform obstacle detection if there are points in the pc
        if pc is not None:
            # Arrays for values of points in each direction
            forward = pc[:, PointCloudProcessor.XYZ["FORWARD"]]
            right = pc[:, PointCloudProcessor.XYZ["RIGHT"]]
            down = pc[:, PointCloudProcessor.XYZ["DOWN"]]
            steps, dists = self.to_laser_scan_data(forward, right)

            # Create matrix where steps, dists, and down arrays are the columns
            directions = np.column_stack((steps, dists, down))
            # Sort rows by first column (steps)
            directions = directions[directions[:, 0].argsort()]
            # Group rows by step
            directions = np.split(
                directions,
                np.unique(directions[:, 0], return_index=True)[1][1:],
                axis=0,
            )

            # Loop through the rows for each step and find obstacles
            for direction in directions:
                # Sort rows for this step by the second column (dist)
                direction = direction[direction[:, 1].argsort()]

                # Step is first column of any row
                step = int(direction[0, 0])

                # Slice the down and dist arrays to do vectorized operations
                # at idx and idx-1
                down1 = direction[:-1, 2]
                down2 = direction[1:, 2]
                dist1 = direction[:-1, 1]
                dist2 = direction[1:, 1]

                # Calculate slope for each pair of points
                drop = np.subtract(down2, down1)
                hike = np.subtract(dist2, dist1)
                slope = np.abs(np.divide(drop, hike))

                # Find first index of row where the value crosses thresholds
                cond_hike = hike > self.min_hole_diameter
                cond_slope = slope > self.max_slope
                index_hike = cond_hike.argmax() if cond_hike.any() else None
                index_slope = cond_slope.argmax() if cond_slope.any() else None

                # Populate laserscan with closest point detected
                if (
                    index_hike is not None
                    and direction[index_hike, 1] <= self.max_obstacle_dist
                ):
                    hike_ranges[step] = direction[index_hike, 1]
                if index_slope is not None:
                    slope_ranges[step] = direction[index_slope, 1]

                # Combine above laserscans
                min_ranges[step] = np.nanmin(
                    (hike_ranges[step], slope_ranges[step]))

            self.hike_pub.publish(self.create_laser_scan(hike_ranges))
            self.slope_pub.publish(self.create_laser_scan(slope_ranges))
            self.combined_pub.publish(self.create_laser_scan(min_ranges))

    """Returns laserscan indices and dists based on the right/forward of a pc"""

    def to_laser_scan_data(self, forward, right):
        # Multiply angles by -1 to get counterclockwise (right to left) ordering
        angles = np.negative(np.arctan2(right, forward))
        # Group angles to discrete indices in laserscan array
        steps = np.divide(
            np.subtract(angles, self.angle_min), self.angle_increment
        ).astype(int)
        # Find the distance each forward, right coordinate from the robot
        dists = np.sqrt(np.add(np.square(forward), np.square(right)))
        return steps, dists


"""Initializes obstacle detection."""


def obstacle_detection(
    max_angle,
    max_obstacle_dist,
    min_hole_diameter,
    scan_time=1.0 / 30,
    range_min=0.105,
    range_max=10.0,
):
    """ Usage:
    od = ObstacleDetector(
        max_angle, max_obstacle_dist, min_hole_diameter, scan_time, range_min, range_max
    )
    """

    rospy.spin()


if __name__ == "__main__":
    try:
        obstacle_detection()
    except (AttributeError, ValueError):
        pass
