#!/usr/bin/env python
import gdal
import ogr
import osr
import sys
import struct
import numpy as np
import get_coords
from scipy.signal import argrelextrema

""" Print all elevation values from dem into txt file """
def printOriginElev(elev_arr, out_file):
    num_rows = elev_arr.shape[0]
    num_cols = elev_arr.shape[1]
    for i in range(0, num_rows):
        for j in range(0, num_cols):
            print(elev_arr[i,j], end=' ', file=out_file)
        print("", file=out_file)

def main():
    print("For: {}, extract elev data in: {}".format(sys.argv[1], sys.argv[2]))

    extr_data = open(sys.argv[2], "w")
    #local_max = open(sys.argv[3], "w")

    dataset = gdal.Open(sys.argv[1], gdal.GA_ReadOnly)
    if not dataset:
        print("ERROR: CANNOT OPEN DEM FILE")
    elif not extr_data:
        print("ERROR: CANNOT OPEN EXTRACT DATA OUTPUT FILE")
    else:
        # Contains metadata and pointer to channels (i.e. rgb) of DEM
        geotransform = dataset.GetGeoTransform()

        if geotransform:

            num_cols = dataset.RasterXSize
            num_rows = dataset.RasterYSize

            # Get coordinates in cartesian
            ext = get_coords.get_corner_coordinates(
                geotransform, num_cols, num_rows
            )

            # Get the coordinate systems to convert
            src_spa_ref_sys = osr.SpatialReference()
            src_spa_ref_sys.ImportFromWkt(dataset.GetProjection())
            targ_spa_ref_sys = src_spa_ref_sys.CloneGeogCS()

            # Convert coordinates to lat and long (decimal not degree minutes
            # seconds)
            geo_ext = get_coords.reproj_coordinates(
                ext, src_spa_ref_sys, targ_spa_ref_sys
            )

            print(
                "Coordinates of each corner pixel in degree decimal:",
                file=extr_data
            )
            print(geo_ext, file=extr_data)

        # loads up a channel of the image i.e. r from rgb
        band = dataset.GetRasterBand(1)

        min = band.GetMinimum()
        max = band.GetMaximum()

        # if a max and min are not embedded in file
        if not min or not max:
            (min, max) = band.ComputeRasterMinMax(True)


        # stores channel info as a large array, making it a np array allows for
        # functions and modules that come with numpy, we'll need it for
        # sampling and stuff
        # n^2
        rasterArray = np.array(band.ReadAsArray())

        # obtains the value assigned in the file to rep nan
        nodata = band.GetNoDataValue()

        # filters out the nan values from the array likely n^2
        rasterArray = np.ma.masked_equal(rasterArray, nodata)

        # checking that the dimensions of the array are correct
        print(rasterArray.shape, file=extr_data)

        # Print all elevation values from dem into txt file
        printOriginElev(rasterArray, extr_data)

        # freeing up data just in case
        band = None
        dataset = None

if __name__ == "__main__":
    main()
