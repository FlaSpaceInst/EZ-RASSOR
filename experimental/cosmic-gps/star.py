#!/usr/bin/env python
# star.py

from math import radians
from math import degrees
from math import sin
from math import atan


class Star:
    def __init__(self):  # constructor
        # set by add_pixel()
        self.pixel_dict = dict()
        self.intensity = 0.0
        self.min_row = 99999
        self.min_col = 99999
        self.max_row = 0
        self.max_col = 0
        # set by centeroid()
        self.star_center_row = 0.0
        self.star_center_col = 0.0
        # set by set_angles()
        self.angle_of_direction = 0.0
        self.angle_distance_from_center = 0.0

    def add_pixel(self, pixel):
        row = pixel[0][0]
        col = pixel[0][1]
        intensity = pixel[1]
        self.pixel_dict[(row, col)] = intensity
        self.intensity = self.intensity + intensity
        if row < self.min_row:
            self.min_row = row
        if row > self.max_row:
            self.max_row = row
        if col < self.min_col:
            self.min_col = col
        if col > self.max_col:
            self.max_col = col

    def centroid(self):
        # get marginal distribution of rows
        marginal_distribution_rows = []
        col = self.min_col
        while col <= self.max_col:
            col_intensity = 0
            row = self.min_row
            while row <= self.max_row:
                if (row, col) in self.pixel_dict:
                    col_intensity = col_intensity + self.pixel_dict[(row, col)]
                row = row + 1
            marginal_distribution_rows.append(col_intensity)
            col = col + 1
        # get marginal distribution of columns
        marginal_distribution_cols = []
        row = self.min_row
        while row <= self.max_row:
            row_intensity = 0
            col = self.min_col
            while col <= self.max_col:
                if (row, col) in self.pixel_dict:
                    row_intensity = row_intensity + self.pixel_dict[(row, col)]
                col = col + 1
            marginal_distribution_cols.append(row_intensity)
            row = row + 1
        # get the mean of cols
        running_total = 0
        pixel_center = 0.5
        for col_intensity in marginal_distribution_rows:
            running_total += pixel_center * col_intensity
            pixel_center += 1
        mean_i = running_total / self.intensity
        # get the mean of rows
        running_total = 0
        pixel_center = 0.5
        for row_intensity in marginal_distribution_cols:
            running_total += pixel_center * row_intensity
            pixel_center += 1
        mean_j = running_total / self.intensity
        self.star_center_row = mean_j + self.min_row
        self.star_center_col = mean_i + self.min_col
        return (self.star_center_row, self.star_center_col)

    def set_angles(self, image_center_x, image_center_y):
        # find direction angle
        opp = 0.0
        adj = 0.0
        init_direction_angle = 0.0
        if self.star_center_col >= image_center_x and (
            self.star_center_row < image_center_y
        ):
            opp = self.star_center_col - image_center_x
            adj = image_center_y - self.star_center_row
            init_direction_angle = 0.0
        elif self.star_center_col > image_center_x and (
            self.star_center_row >= image_center_y
        ):
            opp = self.star_center_row - image_center_y
            adj = self.star_center_col - image_center_x
            init_direction_angle = 90.0
        elif self.star_center_col <= image_center_x and (
            self.star_center_row > image_center_y
        ):
            opp = image_center_x - self.star_center_col
            adj = self.star_center_row - image_center_y
            init_direction_angle = 180.0
        elif self.star_center_col < image_center_x and (
            self.star_center_row <= image_center_y
        ):
            opp = image_center_y - self.star_center_row
            adj = image_center_x - self.star_center_col
            init_direction_angle = 270.0
        temp_angle = atan(opp / adj)
        self.angle_of_direction = init_direction_angle + degrees(temp_angle)
        # find the angle distance from the center
        query = opp / sin(temp_angle)
        # Calibration Function
        self.angle_distance_from_center = (-0.0000015736242766902 * query * query) + (
            0.019311808618316 * query
        )

    def get_intensity(self):
        return self.intensity

    def get_direction_angle(self):
        return self.angle_of_direction

    def get_distance_angle(self):
        return self.angle_distance_from_center

    def shape(self):
        rows = self.max_row - self.min_row + 1
        cols = self.max_col - self.min_col + 1
        if rows > cols:
            return float(cols) / rows
        else:
            return float(rows) / cols

    # test function
    def show(self):
        print "*****"
        print "Pixels:"
        print self.pixel_dict
        print ""
        print "Intensity:"
        print self.intensity
        print ""
        print "Shape:"
        print self.shape()
        print "_____"

    # test function
    def quick_set_angles(self, aod, adfc):
        self.angle_of_direction = aod
        self.angle_distance_from_center = adfc
