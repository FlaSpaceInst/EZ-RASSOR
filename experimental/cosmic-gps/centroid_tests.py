#!/usr/bin/env python
# star_tests

# input: list of stars
# result: list, subpixel values and intensities

# test: outputs the original stars
# test: outputs the centers an intensities


import numpy as np
import cv2 as cv

from thresh_global import thresh_global
from cluster import cluster
from star import Star
from measurement import Calibration_Function


def main():

    img = cv.imread('d_03_12_20_t_22_16_15.jpg', 1)
    img = cv.cvtColor(img, cv.COLOR_RGB2GRAY)
    img = cv.bilateralFilter(img, 5, 150, 150)
    pixels = thresh_global(img, 50)
    list_of_clusters = cluster(pixels)

    list_of_stars = []
    getkey = lambda item : item[0]
    for clustr in list_of_clusters:
        cluster_center = clustr.centroid()
        cluster_intensity = clustr.get_intensity()
        if clustr.shape() > 0.50:  # should be 0.60
            list_of_stars.append([cluster_intensity, cluster_center, clustr])
    list_of_stars = sorted(list_of_stars, key=getkey, reverse=True)

    #print len(list_of_stars)

    print '***Stars***'
    print ''
    for star in list_of_stars:
        print '*****'
        print 'Center:'
        print star[1]
        print ''
        print 'Cluster:'
        star[2].show()
        print '_____'

if __name__ == "__main__":
    main()
