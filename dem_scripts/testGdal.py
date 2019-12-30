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

def main():
    dataset = gdal.Open(sys.argv[1], gdal.GA_ReadOnly)
    if not dataset:
        print("ERROR: CANNOT OPEN FILE")
    else:
        geotransform = dataset.GetGeoTransform()

        if geotransform:
            # lets you know where the origin starts if viewed in qgis
            print("Origin = {}, {}".format(geotransform[0], geotransform[3]))
            
            # Scale in meters (for our .tif files at least) for each pixel
            print("Pixel Size = ({}, {})".format(geotransform[1], geotransform[5]))
            
            num_cols = dataset.RasterXSize
            num_rows = dataset.RasterYSize
            ext = get_corner_coordinates(geotransform, num_cols, num_rows)
            src_spa_ref_sys = osr.SpatialReference()
            src_spa_ref_sys.ImportFromWkt(dataset.GetProjection())
            targ_spa_ref_sys = src_spa_ref_sys.CloneGeogCS()

            geo_ext = reproj_coordinates(ext, src_spa_ref_sys, targ_spa_ref_sys)
            
            # Gives corner coordinates in degree decimal (not degree minutes seconds etc)
            print(geo_ext)

        # loads up a channel of the image i.e. r from rgb
        band = dataset.GetRasterBand(1)
        # The form in which the data is stored
        print("Band Type={}".format(gdal.GetDataTypeName(band.DataType)))

        min = band.GetMinimum()
        max = band.GetMaximum()
  
        # if a max and min are not embedded in file
        if not min or not max:
            (min, max) = band.ComputeRasterMinMax(True)

        print("Min={:.3f}, Max={:.3f}".format(min, max))

        # confirms that a channel is the size of the image but not necessarily all the info
        print("Band size is {} x {}".format(band.XSize, band.YSize))

        # stores channel info as a large array, making it a np array allows for functions
        # and modules that come with numpy, we'll need it for sampling and stuff
        # n^2
        rasterArray = np.array(band.ReadAsArray())
        
        # obtains the value assigned in the file to rep nan
        nodata = band.GetNoDataValue()

        # filters out the nan values from the array likely n^2
        rasterArray = np.ma.masked_equal(rasterArray, nodata)
        
        # checking that the dimensions of the array are correct
        print(rasterArray.shape)

        # freeing up data just in case
        band = None
        dataset = None

        # both n^2 but checking if filtering out nan did anything
        print("Min is: {} Max is: {}".format(np.min(rasterArray), np.max(rasterArray)))

if __name__ == "__main__":
    main()

  
