#!/usr/bin/env python
# Core Function/Node

import numpy as np
import cv2 as cv

from thresh_local_ratio import thresh_local_ratio
from thresh_global import thresh_global
from cluster import cluster
from star import Star
from star_cat import Star_Cat
from measurement import Calibration_Function
from measurement import calculate_angular_distance
from measurement import determine_coordinate
from measurement import calculate_geographic_position


def cell_gps_core():

    position_latitude = 0.0
    position_longitude = 0.0

    # read the configuarion file

    window_size = 9       # used for thresholding
    image_center_x = 1511.5
    image_center_y = 2015.5
    time_list = [13.0, 14.0, 15.0, 16.0, 17.0, 18.0,
                 19.0, 20.0, 21.0, 22.0, 23.0, 24.0,
                 25.0, 26.0, 27.0, 28.0, 29.0, 30.0]
    actual_gha_aries = [5.631667, 20.67333, 35.715, 50.755, 65.79667,
                        80.838333, 95.878333, 110.92, 125.961667, 141.001667,
                        156.04333, 171.08333, 186.125, 201.1667, 216.20667,
                        231.248333, 246.29, 261.33]

    angle_size = 0.5    # keep angle size (0.5, 0.25 0.125, 0.0625) deg
    fov = 90            # max fov 180 deg
    scope_size = 30

    search_angle = 30.0
    epsilon_converge = 0.01

    # initialize the star reference catalogue
    star_catalogue = Star_Cat( 'star_catalogue_init.txt',
                               angle_size,
                               fov,
                               scope_size )
    # initialize the calibration functions
    gha_aries_calibration = Calibration_Function(time_list,
                                                 actual_gha_aries)

    while (True):

        """ waits here until new image is published """

        img = cv.imread('d_03_12_20_t_22_16_15.jpg', 1)   # pub by camera node
        clock_time = 26.2708333                           # system call

        """ image processing step (camera dependent) """

        # convert the image to grey scale and denoise
        img = cv.cvtColor(img, cv.COLOR_RGB2GRAY)
        img = cv.bilateralFilter(img, 5, 150, 150)

        """ Star templating """

        # threshold the image pixels then
        # cluster these pixels into individual stars
        pixels = thresh_global(img, 50)
        list_of_clusters = cluster(pixels)

        # for each cluster find the center, confirm the shape is star-like then
        # for each of the stars calculate its angles (direction and distance)
        # from the image center finally pair each star object with its
        # intensity and sort the pairs by intensity
        list_of_stars = []
        getkey = lambda item : item[0]
        for clustr in list_of_clusters:
            if clustr.shape() > 0.50:
                cluster_intensity = clustr.get_intensity()
                cluster_center = clustr.centroid()
                clustr.set_angles(image_center_x, image_center_y)
                list_of_stars.append([cluster_intensity, clustr])
        list_of_stars = sorted(list_of_stars, key=getkey, reverse=True)

        """ Star Matching """
        
        count = 0
        list_of_stars_size = len(list_of_stars)
        list_of_possible_matches = []

        # loop five times enforced by count
        while (count < 5):

            # break early if too few stars
            if list_of_stars_size < 5:
                break;

            star1 = list_of_stars[count+0][1]
            star2 = list_of_stars[count+1][1]
            star3 = list_of_stars[count+2][1]
            star4 = list_of_stars[count+3][1]
            star5 = list_of_stars[count+4][1]
            stars = (star1, star2, star3, star4, star5)

            # Try matching to the scope for the first try
            if count == 0:
                list_of_possible_matches = star_catalogue.match_scope(stars)
            else:
                list_of_possible_matches = star_catalogue.match_global(stars)

            # if no candidates are returned, remove the count+0
            # star with the next brightest
            count += 1
            list_of_stars_size -= 1
            if len(list_of_possible_matches) > 0:
                break # can be possibly many group of candidates.

        # Position derivation
        # find the GHA using the time and calibration curve
        gha_aries = gha_aries_calibration.neville_interpolation(clock_time)
        # using the candidate stars derive the celestial position
        min_d = 360.0
        for match in list_of_possible_matches:
            # take first three of match calculate the celestial position
            lamb1 = match[0][2]
            phi1 = match[0][3]
            d1 = match[0][4]
            lamb2 = match[1][2]
            phi2 = match[1][3]
            d2 = match[1][4]
            lamb3 = match[2][2]
            phi3 = match[2][3]
            d3 = match[2][4]
            coor = determine_coordinate(lamb1, phi1, d1,
                                        lamb2, phi2, d2,
                                        lamb3, phi3, d3,
                                        search_angle, epsilon_converge)
            gp = calculate_geographic_position(coor, gha_aries)
            d = calculate_angular_distance(position_longitude,
                                           position_latitude, gp[0], gp[1])
            if d < min_d:
                save_coor = coor
                save_gp = gp
                save_match = match
                min_d = d
        position_latitude = save_gp[1]
        position_longitude = save_gp[0]
        star_catalogue.rescope(coor[0], coor[1])
        print 'Geographic Position', save_gp
        print save_match
        break;
    # end


if __name__ == "__main__":
    cell_gps_core()
