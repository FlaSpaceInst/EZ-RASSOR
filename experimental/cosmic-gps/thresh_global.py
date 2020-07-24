#!/usr/bin/env python
# thresh_global.py

# This module holds diffenent functions for thresholding an image
# input: denoised gray scale image
# output: tuple with dictionary of active pixels

# *********************#
# method not optimized #
# *********************#


def thresh_global(img, thresh_value=25):
    img_dimension = img.shape
    rows = img_dimension[0]
    cols = img_dimension[1]

    # add bright pixels to dictionary
    bag_of_pixels = dict()
    row = 0
    # scanning rows
    while row < rows:
        col = 0
        # scanning cols
        while col < cols:
            # set threshold values to dictionary
            pixel_intensity = img.item(row, col)
            if pixel_intensity >= thresh_value:
                bag_of_pixels[(row, col)] = pixel_intensity
            col = col + 1
        row = row + 1

    return bag_of_pixels
