#!/usr/bin/env python
# cluster.py

# input: dictionary of pixel coordinates and intensities
# outupt: a list of stars


from star import Star


def cluster(pixels):
    list_of_star_objects = []
    temp_stack = []

    while len(pixels) != 0:
        new_star_object = Star()
        temp_stack.append(pixels.popitem())
        while len(temp_stack) != 0:
            pixel = temp_stack.pop()
            row = pixel[0][0]
            col = pixel[0][1]
            # check neighbors
            row2 = row + 1
            row1 = row - 1
            col2 = col + 1
            col1 = col - 1
            if (row2, col2) in pixels:
                temp_stack.append(((row2, col2), pixels.pop((row2, col2))))
            if (row2, col) in pixels:
                temp_stack.append(((row2, col), pixels.pop((row2, col))))
            if (row2, col1) in pixels:
                temp_stack.append(((row2, col1), pixels.pop((row2, col1))))
            if (row, col2) in pixels:
                temp_stack.append(((row, col2), pixels.pop((row, col2))))
            if (row, col1) in pixels:
                temp_stack.append(((row, col1), pixels.pop((row, col1))))
            if (row1, col2) in pixels:
                temp_stack.append(((row1, col2), pixels.pop((row1, col2))))
            if (row1, col) in pixels:
                temp_stack.append(((row1, col), pixels.pop((row1, col))))
            if (row1, col1) in pixels:
                temp_stack.append(((row1, col1), pixels.pop((row1, col1))))
            new_star_object.add_pixel(pixel)
        list_of_star_objects.append(new_star_object)

    return list_of_star_objects

