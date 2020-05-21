# show_image.py

import numpy as np
import cv2 as cv

def main():

    img = cv.imread('d_03_12_20_t_22_16_15.jpg', 1)
    img = cv.cvtColor(img, cv.COLOR_RGB2GRAY)
    img = cv.bilateralFilter(img, 5, 150, 150)

    screen_res = (1280, 720)
    scale_width = screen_res[0] / img.shape[1]
    scale_height = screen_res[1] / img.shape[0]
    scale = min(scale_width, scale_height)

    window_width = int(img.shape[1] * scale)
    window_height = int(img.shape[0] * scale)

    cv.namedWindow('Resized Window', cv.WINDOW_NORMAL)
    cv.resizeWindow('Resized Window', window_width, window_height)

    cv.imshow('Resized Window', img)
    cv.waitKey(0)
    cv.destroyAllWindows()

if __name__ == "__main__":
    main()
