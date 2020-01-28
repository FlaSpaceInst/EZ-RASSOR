# clust.py

# This module holds diffenent functions for clustering a bag of pixels
# input: dictionary of pixel coordinates and intensities
# outupt: a list of stars

# *********************#
# method not optimized #
# *********************#

from star import Star


def cluster(pixls):
    slist = []
    stack = []

    # loop while pixls
    while len(pixls) != 0:
        tmpStar = Star()  # new star
        stack.append(pixls.popitem())
        while len(stack) != 0:
            px = stack.pop()
            row = px[0][0]
            col = px[0][1]
            intensity = px[1]
            # check neighbors
            row2 = row + 1
            row1 = row - 1
            col2 = col + 1
            col1 = col - 1
            if (row2, col2) in pixls:
                stack.append(((row2, col2), pixls.pop((row2, col2))))
            if (row2, col) in pixls:
                stack.append(((row2, col), pixls.pop((row2, col))))
            if (row2, col1) in pixls:
                stack.append(((row2, col1), pixls.pop((row2, col1))))
            if (row, col2) in pixls:
                stack.append(((row, col2), pixls.pop((row, col2))))
            if (row, col1) in pixls:
                stack.append(((row, col1), pixls.pop((row, col1))))
            if (row1, col2) in pixls:
                stack.append(((row1, col2), pixls.pop((row1, col2))))
            if (row1, col) in pixls:
                stack.append(((row1, col), pixls.pop((row1, col))))
            if (row1, col1) in pixls:
                stack.append(((row1, col1), pixls.pop((row1, col1))))
            tmpStar.add(px)
        # no more neighbor px
        slist.append(tmpStar)

    return slist

