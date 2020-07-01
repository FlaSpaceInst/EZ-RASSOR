#!/usr/bin/env python
# thresh_local_ratio.py

# This module holds a function for thresholding an image.
# input: denoised gray scale image
# output: tuple with dictionary of active pixels


import numpy as np
import cv2 as cv


def thresh_local_ratio(img, window_size=9):
    half_window_size = (window_size - 1) / 2  # always even
    window_size_square = window_size ** 2
    img_dimension = img.shape
    rows = img_dimension[0]
    cols = img_dimension[1]

    thresh_array = np.zeros((rows, cols))

    rows = rows - half_window_size - 1
    cols = cols - half_window_size - 1

    max_value = 100.0
    min_value = 0.0
    max_ratio_value = min_value
    min_ratio_value = max_value

    row = half_window_size
    # scanning rows
    while row <= rows:
        col = half_window_size
        # scanning cols
        while col <= cols:
            window_intensity = 0.0
            r = row - half_window_size
            sr = row + half_window_size
            # scanning s rows
            while r <= sr:
                c = col - half_window_size
                sc = col + half_window_size
                # scanning s cols
                while c <= sc:
                    window_intensity = window_intensity + img.item(r, c)
                    c = c + 1
                r = r + 1
            avg_window_intensity = window_intensity / window_size_square
            if avg_window_intensity == 0.0:
                ratio_value = 0.0
            else:
                ratio_value = img.item(row, col) / avg_window_intensity
            thresh_array.itemset((row, col), ratio_value)
            # update max and min ratio
            if ratio_value > max_ratio_value:
                max_ratio_value = ratio_value
            if ratio_value < min_ratio_value:
                min_ratio_value = ratio_value
            col = col + 1
        row = row + 1
    thresh_value = (max_ratio_value + min_ratio_value) / 2

    # add bright pixels to dictionary
    bag_of_pixels = dict()
    row = half_window_size
    # scanning rows
    while row <= rows:
        col = half_window_size
        # scanning cols
        while col <= cols:
            # set threshold values to dictionary
            ratio_value = thresh_array.item(row, col)
            if ratio_value >= thresh_value:
                pixel_intensity = img.item(row, col)
                bag_of_pixels[(row, col)] = pixel_intensity
            col = col + 1
        row = row + 1

    return bag_of_pixels
