# threshGlobal.py

# This module holds diffenent functions for thresholding an image
# input: denoised gray scale image
# outupt: tuple with dictionary of active pixels

# *********************#
# method not optimized #
# *********************#

import random
import numpy as np
import cv2 as cv


def threshGlobal(img, randSiz=25):
    imgDimension = img.shape
    rows = imgDimension[0]
    cols = imgDimension[1]
    magic = 2  # wiggle number

    threshTemp = 0
    for x in range(0, randSiz):
        randR = random.randint(0, rows)
        randC = random.randint(0, cols)
        threshTemp = threshTemp + img.item(randR, randC)

    threshVal = (float(threshTemp) / randSiz) * magic

    # add bright pixels to dictionary
    pixelBag = dict()
    row = 0
    # scanning rows
    while row < rows:
        col = 0
        # scanning cols
        while col < cols:
            # set threshold values to dictionary
            pixIntensity = img.item(row, col)
            if pixIntensity >= threshVal:
                pixelBag[(row, col)] = pixIntensity
            col = col + 1
        row = row + 1

    return pixelBag
