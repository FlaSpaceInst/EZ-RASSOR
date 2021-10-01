#!/usr/bin/env python

import rospy
import math
import numpy as np
from collections import defaultdict
import cv2

from ezrassor_swarm_control.srv import GetRoverStatus, PreemptPath


def euclidean_distance(x1, x2, y1, y2):
    """ Calculate Euclidean distance from (x1,y1) to (x2,y2). """

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

#Leveling Algorithm Functions

def dig_dump(matrix, dig_locations, dump_locations, dig_location, dump_location):
    """
    Dig and dump between two locations until one location is level.
    """

    count = 0
    while dig_locations[dig_location] and dump_locations[dump_location]:

        # If a location does not have enough resources, take resources available.
        dig_amount = 1 if dig_locations[dig_location] >= 1 else dig_locations[dig_location]
        dig_amount = dig_amount if dump_locations[dump_location] * -1  >= dig_amount else dump_locations[dump_location] * -1
        dig_locations[dig_location] -= dig_amount
        dump_locations[dump_location] += dig_amount

        dig_y, dig_x = dig_location
        dump_y, dump_x = dump_location

        matrix[dig_y][dig_x] -= dig_amount
        matrix[dump_y][dump_x] += dig_amount
        count += dig_amount
    return count

# Threshold value for a cell to be considered level.

threshold = 0.09
def find_largest_difference(dig_locations, dump_locations):
    """
    Pair the dig annd dump pairs which seem like the best choice. (Greedy component)
    Return dig and dump pairs which have the largest distance difference between its closest dump location and its next closest dump location.
    If the closest dump location to chosen dig location is paired off with another dig location, then the overall pathing distance will be greater.
    """

    # TODO: Use dynamic programming.

    # Get the distances between every dig and dump pair.
    distance_matrix = []
    for dig_location in dig_locations:
        if dig_locations[dig_location] < threshold:
            continue
        distance_matrix.append(sorted([(euclidean_distance(dig_location[0], dig_location[1], dump_location[0], dump_location[1]), dig_location, dump_location) for dump_location in dump_locations if abs(dump_locations[dump_location]) > threshold]))

    # Get the dig locations which are the farthest apart between their closest dump location and their next closest dump location.
    # TODO: Account for edge case when all largest distance difference variables are the same.
    #       Solution is to recalculate largest_difference_list where the closest path distance is subtracted by a path distance with a value other then the next closest path distance.
    largest_differences_list = []
    for dig_location_distances in distance_matrix:
        if len(dig_location_distances) == 1:
            largest_differences_list.append((0, dig_location_distances[0][1], dig_location_distances[0][2]))
        else:
            largest_differences_list.append((dig_location_distances[1][0] - dig_location_distances[0][0], dig_location_distances[0][1], dig_location_distances[0][2]))
    largest_differences_list.sort()

    return largest_differences_list[-1]

def level_ground(matrix):
    """
    Pair digging and dumping locations to level a given matrix using a greedy approach.
    """

    # Holds dig or dump coordinates  and their elevation.
    dig_locations = {}
    dump_locations = {}

    # Pairs dig and dump location and specifies how many actions must be completed in both locations.
    dig_dump_pairs = defaultdict(list)

    # Iterate through the entire matrix, adding digging sites and dumping sites to corresponding dictionary.
    for i in range(len(matrix)):
        for j in range(len(matrix[i])):
            if matrix[i][j] > threshold:
                dig_locations[(i,j)] = matrix[i][j]
            elif matrix[i][j] < -threshold:
                dump_locations[(i,j)] = matrix[i][j]

    def check_level():
        # Check if the area is level.
        # One location is allowed to contain the excess resource of the matrix.
        for row in matrix:
            for column in row:
                if abs(column) > threshold:
                    return False
        return True

    while not check_level():
        # Pair dig and dump locations until all locations are level.
        distance, dig_location, dump_location = find_largest_difference(dig_locations, dump_locations) # Greedy choice
        number_of_actions = dig_dump(matrix, dig_locations, dump_locations, dig_location, dump_location)
        dig_dump_pairs[dig_location].append((dump_location, number_of_actions)) # Pair

    # Convert output points to objects and coordinates to simulation coordinates.
    class Point():
        def __init__(self, x, y):
            self.x = x
            self.y = y

    # Convert image coordinates to simulation coordinates in a Point object.
    def convert_to_simulation_Point(matrix, x, y):
        x -= len(matrix[0])) // 2
        y = - (y - (len(matrix) // 2))
        return Point(x, y)

    converted_dig_dump_pairs = defaultdict(list)

    for dig_location in dig_dump_pairs:
        dig_location_point = convert_to_simulation_Point(matrix, dig_location[0], dig_location[1])

        for dump_location, actions in dig_dump_pairs[dig_location]:
            dump_location_point = convert_to_simulation_Point(matrix, dump_location[0], dump_location[1])

            converted_dig_dump_pairs[dig_location_point].append((dump_location_point, actions))

    return matrix, converted_dig_dump_pairs

def convert_image(map_path, pixel_scale):
    # read in image
    moon_img = cv2.imread(map_path, 0)
    # convert image to float32 and extract tuple
    height_matrix = moon_img.astype(np.float32)
    img_height, img_width = height_matrix.shape

    # calculate the mean
    level_elevation = np.mean(height_matrix)

    # elevation change from each rover dig/dump
    standard_dig_amount = 1

    # loop through array and standardize the array
    for row in range(0, img_width):
        for col in range(0, img_height):
            height_matrix[row][col] = ((height_matrix[row][col] - level_elevation) / standard_dig_amount)

    rover_standard_dig_height = 5
    rover_standard_dig_width = 10

    # Number of rows in new sub matrix that is replacing one cell of original matrix
    scale_height = pixel_scale // rover_standard_dig_height
    # Number of columns in new sub matrix that is replacing one cell of original matrix
    scale_width = pixel_scale // rover_standard_dig_width

    # Dimensions of expanded matrix
    scaled_array_width = img_width * scale_width
    scaled_array_height = img_height * scale_height
    scaled_array = np.zeros((scaled_array_height, scaled_array_width))

    for row in range(0, img_width):
        for col in range(0, img_height):
            for x in range(row * scale_height, (row * scale_height) + scale_height):
                for y in range(col * scale_width, (col * scale_width) + scale_width):
                    scaled_array[x][y] = height_matrix[row][col]

    return scaled_array

def pixel_normalize(map_path):
    # read in image
    moon_img = cv2.imread(map_path, 0)
    # convert image to float32 and extract tuple
    height_matrix = moon_img.astype(np.float32)
    img_height, img_width = height_matrix.shape

    # calculate the mean
    level_elevation = np.mean(height_matrix)

    # elevation change from each rover dig/dump
    standard_dig_amount = 1

    # loop through array and standardize the array
    for row in range(0, img_width):
        for col in range(0, img_height):
            height_matrix[row][col] = ((height_matrix[row][col] - level_elevation) / standard_dig_amount)

    return height_matrix

def pixel_expand(map_path, pixel_scale):
    # read in image
    moon_img = cv2.imread(map_path, 0)
    # convert image to float32 and extract tuple
    height_matrix = moon_img.astype(np.float32)
    img_height, img_width = height_matrix.shape

    # calculate the mean
    level_elevation = np.mean(height_matrix)
    
    # elevation change from each rover dig/dump
    standard_dig_amount = 1

    rover_standard_dig_height = 5
    rover_standard_dig_width = 10
    
    # Number of rows in new sub matrix that is replacing one cell of original matrix
    scale_height = pixel_scale // rover_standard_dig_height
    # Number of columns in new sub matrix that is replacing one cell of original matrix
    scale_width = pixel_scale // rover_standard_dig_width
    
    # Dimensions of expanded matrix
    scaled_array_width = img_width * scale_width
    scaled_array_height = img_height * scale_height
    scaled_array = np.zeros((scaled_array_height, scaled_array_width))
    
    for row in range(0, img_width):
        for col in range(0, img_height):
            for x in range(row * scale_height, (row * scale_height) + scale_height):
                for y in range(col * scale_width, (col * scale_width) + scale_width):
                    scaled_array[x][y] = height_matrix[row][col]
    
    return scaled_array
