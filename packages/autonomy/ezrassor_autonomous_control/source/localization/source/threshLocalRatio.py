# threshLocalRatio.py

# This module holds diffenent functions for thresholding an image.
# input: denoised gray scale image
# outupt: tuple with dictionary of active pixels

# *********************#
# method not optimized #
# *********************#

import numpy as np
import cv2 as cv


def threshLocalRatio(img, winSiz=9):
    hlfWin = (winSiz - 1) / 2  # always even
    winSiz2 = winSiz ** 2
    imgDimension = img.shape
    rows = imgDimension[0]
    cols = imgDimension[1]

    thrsh = np.zeros((rows, cols))

    rows = rows - hlfWin - 1
    cols = cols - hlfWin - 1

    maxVal = 100
    minVal = 0
    maxRatioVal = minVal
    minRatioVal = maxVal

    row = hlfWin
    # scanning rows
    while row <= rows:
        col = hlfWin
        # scanning cols
        while col <= cols:
            windowInt = 0
            r = row - hlfWin
            sr = row + hlfWin
            # scanning s rows
            while r <= sr:
                c = col - hlfWin
                sc = col + hlfWin
                # scanning s cols
                while c <= sc:
                    windowInt = windowInt + img.item(r, c)
                    c = c + 1
                r = r + 1
            avgWinInt = float(windowInt) / winSiz2
            if avgWinInt == 0:
                ratioVal = 0
            else:
                ratioVal = img.item(row, col) / avgWinInt
            thrsh.itemset((row, col), ratioVal)
            # update max and min ratio
            if ratioVal > maxRatioVal:
                maxRatioVal = ratioVal
            if ratioVal < minRatioVal:
                minRatioVal = ratioVal
            col = col + 1
        row = row + 1
    threshVal = (maxRatioVal + minRatioVal) / 2

    # add bright pixels to dictionary
    pixelBag = dict()
    row = hlfWin
    # scanning rows
    while row <= rows:
        col = hlfWin
        # scanning cols
        while col <= cols:
            # set threshold values to dictionary
            ratVal = thrsh.item(row, col)
            if ratVal >= threshVal:
                pixIntensity = img.item(row, col)
                pixelBag[(row, col)] = pixIntensity
            col = col + 1
        row = row + 1

    return pixelBag
