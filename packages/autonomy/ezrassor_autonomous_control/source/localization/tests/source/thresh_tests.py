# thresh_tests
# 12/13/2019

# input: 2D array with intensity values, size
# result: queue, threshold array with 1s and 0s

# test: outputs the original array with intensities
# test: outputs the threshold image based on the dictionary


import numpy as np
import cv2 as cv
from threshLocalAbs import threshLocalAbs


def main():
    # get test img convert to grey scale
    img = cv.imread('star_small.png', 1)
    img = cv.cvtColor(img, cv.COLOR_RGB2GRAY)
    ftr = cv.bilateralFilter(img, 5, 150, 150)

    imgDimension = img.shape
    rows = imgDimension[0]
    cols = imgDimension[1]

    thrD = threshLocalAbs(ftr, 5)
    print thrD
    thrA = cvtDict(thrD, rows, cols)

    cv.imshow('img', img)
    cv.imshow('ftr', ftr)
    cv.imshow('tst', thrA)
    cv.waitKey(0)
    cv.destroyAllWindows()

def cvtDict(thr, rows, cols):
    tst = np.zeros((rows, cols))
    # loop through dictionary
    for x in thr:
        tst.itemset((x[0], x[1]), thr[x])
    return tst

if __name__ == "__main__":
    main()
