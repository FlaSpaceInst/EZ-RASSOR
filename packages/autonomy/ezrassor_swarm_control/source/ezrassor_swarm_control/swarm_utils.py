#!/usr/bin/env python

import rospy
import math
import numpy as np
from collections import defaultdict
from geometry_msgs.msg import Point
import cv2

from ezrassor_swarm_control.srv import GetRoverStatus, PreemptPath


def euclidean_distance(x1, x2, y1, y2):
    """Calculate Euclidean distance from (x1,y1) to (x2,y2)."""

    return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)


def euclidean2D(a, b):
    """
    Returns the 2D (x and y axes) euclidean distance between 2 ROS Points
    """

    return np.sqrt((b.x - a.x) ** 2 + (b.y - a.y) ** 2)


def get_rover_status(rover_num):
    """
    ROS service that allows the swarm controller to request and
    receive a given rover's current position
    """

    # Build the service endpoint
    service = "/ezrassor{}/rover_status".format(rover_num)

    # Ensure it's running
    rospy.wait_for_service(service, 5.0)

    try:
        # Retrieve rover status
        get_status = rospy.ServiceProxy(service, GetRoverStatus)
        response = get_status()

        # Convert float coordinates to integers
        response.pose.position.x = int(round(response.pose.position.x))
        response.pose.position.y = int(round(response.pose.position.y))
        return response

    except rospy.ServiceException as e:
        print("Service call failed: {}".format(e))


def preempt_rover_path(rover_num):
    """
    ROS service that allows the swarm controller to force a rover to
    stop following a path
    """

    # Build the service endpoint
    service = "/ezrassor{}/preempt_path".format(rover_num)

    # Ensure it's running
    rospy.wait_for_service(service, 5.0)

    try:
        # Retrieve rover status
        preempt_path = rospy.ServiceProxy(service, PreemptPath)
        response = preempt_path()

        return response

    except rospy.ServiceException as e:
        print("Service call failed: {}".format(e))


# Leveling Algorithm Functions


def create_level_instructions(matrix):
    """Given a matrix where each cell is a point on the map and each cell's value is the
    elvation at that point,
    create a set of instructions which describe the redistrubtion of elevation at
    specific points to make every cell have
    almost the same elevation.
    Assume that the elevation may be distributed in such a way that resulting area may
    be level."""

    # Elevation at which the location is considered level.
    threshold = 0.0000000009

    # Holds a point on the map (x, y) whose elevation is greater then desired.
    # Key is point and value is elevation.
    dig_locations = {}
    # Like dig locations but elevation of points must be less then desired.
    dump_locations = {}
    # Holds dig locations which have not been included in enough instructions to be
    # considered level.
    dig_set = set()
    dump_set = set()  # Like dig_set.
    # Holds every instruction created.
    # Organized by dig location
    # Key: dig_location
    # Value: A list of tuples of format (dump_location, elevation_change)
    dig_dump_pairs = defaultdict(list)

    # Iterate through the matrix and find every dig location and dump location
    # in the area.
    # Populate respective variables.
    for i in range(len(matrix)):
        for j in range(len(matrix[i])):
            # Location has an elevation greater then what is considered level.
            if matrix[i][j] > threshold:
                dig_locations[(i, j)] = matrix[i][j]
                dig_set.add((i, j))
            # Location has an elevation then what is considered level.
            elif matrix[i][j] < -threshold:
                dump_locations[(i, j)] = matrix[i][j]
                dump_set.add((i, j))

    # Create list where every element is a list which contains the distances between one
    # dig location and all dump locations.
    distance_matrix = defaultdict(list)
    for dig_location in dig_locations:
        for dump_location in dump_locations:
            # Append a tuple which contains:
            # 1. distance between the current dig location and dump location
            # 2. the coordinates of the dump location
            distance_matrix[dig_location].append(
                (
                    euclidean_distance(
                        dig_location[0],
                        dump_location[0],
                        dig_location[1],
                        dump_location[1],
                    ),
                    dump_location,
                )
            )
        # Sort the list so that the dump locations which are closest to the current dig
        # location are first.
        distance_matrix[dig_location].sort()

    # Create an instruction while there are still dig locations and dump locations to
    # be leveled.
    while dig_set and dump_set:
        # Create an instruction based on the following definitions and procedure:
        # A. For each dig location, take the difference between:
        #   1. The distance between the dig location and the closest available dump
        #   location
        #   2. The distance between the dig location and the next closest available dump
        #   location
        # B. Choose the dig location and the closest dump location which provided the
        # largest value in step A.
        # Note that the largest value in step A is refered as the largest difference.
        sub_pair = None
        largest_difference = None
        for dig_location in distance_matrix:

            distance_list = distance_matrix[dig_location]

            # Delete closest dump locations not available.
            while distance_list and distance_list[0][1] not in dump_set:
                del distance_list[0]

            # Delete next closest dump locations not avaiable.
            while (
                len(distance_list) > 1 and distance_list[1][1] not in dump_set
            ):
                del distance_list[1]

            # If there are no dump locations to level dig location, raise exception.
            if not distance_list:
                raise Exception

            # If no largest_difference has been set, set it.
            if not largest_difference:
                # If there is only one dump location, then the difference is 0.
                if len(distance_list) == 1:
                    largest_difference = 0
                else:
                    largest_difference = (
                        distance_list[1][0] - distance_list[0][0]
                    )
                sub_pair = (dig_location, distance_list[0][1])

            # Only set this pair as sup pair if the difference between the closest dump
            # location and the next closest dump location is greater then the largest
            # distance difference recorded.
            elif (
                len(distance_list) > 1
                and largest_difference
                < distance_list[1][0] - distance_list[0][0]
            ):
                sub_pair = (dig_location, distance_list[0][1])
                largest_difference = distance_list[1][0] - distance_list[0][0]

        # Level subpair with largest difference.
        dig_location, dump_location = sub_pair
        dig_elevation = dig_locations[dig_location]
        dump_elevation = abs(dump_locations[dump_location])
        # The change of elevation is the elvation which is closest to the desired
        # elevation.
        elevation_change = (
            dig_elevation if dig_elevation < dump_elevation else dump_elevation
        )

        # Update elevation of both locations in sup-pair.
        dig_locations[dig_location] -= elevation_change
        dump_locations[dump_location] += elevation_change

        # If a location is level, remove it from set.
        if dig_locations[dig_location] <= threshold:
            dig_set.remove(dig_location)
            del distance_matrix[dig_location]

        if dump_locations[dump_location] >= -threshold:
            dump_set.remove(dump_location)

        # Create new instruction.
        dig_dump_pairs[dig_location].append((dump_location, elevation_change))

    # Convert image coordinates to simulation coordinates in a Point object.
    def convert_to_simulation_Point(matrix, x, y):
        x -= len(matrix[0]) // 2
        y = -(y - (len(matrix) // 2))
        a_point = Point()
        a_point.x = x
        a_point.y = y
        return a_point

    converted_dig_dump_pairs = defaultdict(list)

    for dig_location in dig_dump_pairs:
        dig_location_point = convert_to_simulation_Point(
            matrix, dig_location[0], dig_location[1]
        )

        for dump_location, actions in dig_dump_pairs[dig_location]:
            dump_location_point = convert_to_simulation_Point(
                matrix, dump_location[0], dump_location[1]
            )

            converted_dig_dump_pairs[dig_location_point].append(
                (dump_location_point, actions)
            )
    return converted_dig_dump_pairs


# Image Processing functions


def convert_image(map_path, pixel_scale):
    # read in image
    moon_img = cv2.imread(map_path, 0)
    # convert image to float32 and extract tuple
    height_matrix = moon_img.astype(np.float32)
    img_height, img_width = height_matrix.shape
    print("Original dim: {},{}".format(img_height, img_width))

    # rover's interesting dimensions
    rover_standard_dig_height = 1
    rover_standard_dig_width = 2

    # Number of rows in new sub matrix that is replacing one cell of original matrix
    scale_height = pixel_scale // rover_standard_dig_height
    # Number of columns in new sub matrix that is replacing one cell of original matrix
    scale_width = pixel_scale // rover_standard_dig_width

    # Dimensions of expanded matrix
    scaled_array_width = img_width * scale_width
    scaled_array_height = img_height * scale_height
    dim = (scaled_array_height, scaled_array_width)

    resized = cv2.resize(moon_img, dim, interpolation = cv2.INTER_AREA)

    print('Resized Dimensions : ',resized.shape)

    cv2.imwrite("nasa_moon_dem.jpg", resized)
 
    cv2.imshow("Resized image", resized)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

    return None


def pixel_normalize(height_matrix):
    # elevation change from each rover dig/dump
    standard_dig_amount = 1

    mean = height_matrix.mean()

    height_matrix = (height_matrix - mean) / standard_dig_amount

    return height_matrix