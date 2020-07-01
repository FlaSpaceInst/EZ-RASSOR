#!/usr/bin/env python

""" Program to check the size of a dem """

import gdal
import ogr
import osr
import sys

def main():
    # Open file
    dataset = gdal.Open(sys.argv[1], gdal.GA_ReadOnly)
    if not dataset:
        print("-1 -1")
    else:
        # Get metadata
        geotransform = dataset.GetGeoTransform()

        if geotransform:
            num_cols = dataset.RasterXSize
            num_rows = dataset.RasterYSize
            print("{} {}".format(num_cols, num_rows))
        else:
        	print("-1 -1")

if __name__ == "__main__":
    main()
