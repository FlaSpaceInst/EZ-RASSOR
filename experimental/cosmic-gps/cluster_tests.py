#!/usr/bin/env python
# cluster_tests

# input: dictionary with pixel coordinates and intensities
# result: list, clusted pixels (stars)

# test: outputs the original dictionary
# test: outputs the star clusters


import numpy as np
import cv2 as cv

from thresh_global import thresh_global
from cluster import cluster
from star import Star


def main():

    img = cv.imread('d_03_12_20_t_22_16_15.jpg', 1)
    img = cv.cvtColor(img, cv.COLOR_RGB2GRAY)
    img = cv.bilateralFilter(img, 5, 150, 150)
    pixels = thresh_global(img, 50)

    list_of_clusters = cluster(pixels)

    print '***Clusters***'
    print ''
    for clust in list_of_clusters:
        clust.show()
        print ''


if __name__ == "__main__":
    main()
