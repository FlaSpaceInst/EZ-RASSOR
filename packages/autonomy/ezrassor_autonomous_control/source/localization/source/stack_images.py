# stack_images.py

import numpy as np
import opencv as cv

# take an array of images and combine
# them into a single image.
def stack_images(images):
    # pop first image as starting.
    img_cumulate = images.pop()
    # loop through the images
    for tmp in images:
        # downscale/splice
