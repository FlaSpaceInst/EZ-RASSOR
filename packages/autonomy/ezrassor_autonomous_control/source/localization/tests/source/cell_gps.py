# Test

import numpy as np
import cv2 as cv



# take a colored image
img = cv.imread('star_test.png', 1)
cut = cv.imread('star_small.png', 1)
# convert img to grey scale
imgG = cv.cvtColor(img, cv.COLOR_RGB2GRAY)
# how many channels
# prints (x, y, z)
# where x is rows
#       y is cols
#       z is channels if not gray scale
# print (img.shape)
# reduce noise (preserve edges)
filt = cv.bilateralFilter(imgG, 5, 150, 150)

# cut = img[200:250, 30:130]
# cv.imwrite('star_small.png', cut)


# show results
cv.imshow('Grey', imgG)
cv.imshow('Denoise', filt)
cv.imshow('Cut', cut)
cv.waitKey(0)
cv.destroyAllWindows()
