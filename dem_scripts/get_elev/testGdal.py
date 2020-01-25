#!/usr/bin/env python
import gdal
import ogr
import osr
import sys
import struct
import numpy as np

def get_corner_coordinates(geotransform, num_cols, num_rows):
    corners = []
    x_arr = [0, num_cols]
    y_arr = [0, num_rows]
    for x in x_arr:
        for y in y_arr:
            x_cor = geotransform[0] + (x * geotransform[1]) + (y * geotransform[2])
            y_cor = geotransform[3] + (x * geotransform[4]) + (y * geotransform[5])
            corners.append([x_cor, y_cor])
        y_arr.reverse()
    return corners

def reproj_coordinates(coords, src_spa_ref_sys, targ_spa_ref_sys):
    trans_coords = []
    transform = osr.CoordinateTransformation(src_spa_ref_sys, targ_spa_ref_sys)
    for x, y in coords:
        x, y, z = transform.TransformPoint(x,y)
        trans_coords.append([x,y])
    return trans_coords

