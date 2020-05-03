#!/usr/bin/env python
# thresh_tests

# input: 2D array with intensity values, size
# result: queue, threshold array with 1s and 0s

# test: outputs the original array with intensities
# test: outputs the threshold image based on the dictionary


import numpy as np
import cv2 as cv
from thresh_local_ratio import thresh_local_ratio
from thresh_global import thresh_global


def main():

    window_size = 9
    thresh_value = 50

    img = cv.imread('d_03_12_20_t_22_16_15.jpg', 1)
    img = cv.cvtColor(img, cv.COLOR_RGB2GRAY)
    img = cv.bilateralFilter(img, 5, 150, 150)

    rows = img.shape[0]
    cols = img.shape[1]

    #thresh_dict = thresh_local_ratio(img, window_size)
    thresh_dict = thresh_global(img, thresh_value)

    thresh_array = convert_dict_to_array(thresh_dict, rows, cols)
    print thresh_dict
    #cv.imwrite('thresh_test_output.png', thresh_array)

def convert_dict_to_array(thresh_dict, rows, cols):
    temp_array = np.zeros((rows, cols))
    for pixel in thresh_dict:
        temp_array.itemset((pixel[0], pixel[1]), thresh_dict[pixel])
    return temp_array

if __name__ == "__main__":
    main()
