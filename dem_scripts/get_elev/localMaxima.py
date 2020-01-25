#!usr/bin/env python
import gdal
import ogr
import osr
import sys
import struct
import numpy as np
import testGdal
from scipy.signal import argrelextrema

def localMaxima(dataset_arr, threshold, out_file):
    cand_list = (dataset_arr > threshold) * dataset_arr
    #print(cand_list)
    
    # Comprised of two arrays, each for respective x and y coord in cand_list
    maxInd = argrelextrema(cand_list, np.greater)
    #print(maxInd)
    
    # R is a sublist of cand_list that contains all the local maxima values
    r = cand_list[maxInd]
    #print(r)
    #print(r.size)
    num_of_ent = r.size
    diction = {}
    j = 0
    print("Local Maxima coordinates and values:", file=out_file)
    for i in range(0, num_of_ent):
        print("({}, {}) : {}".format(maxInd[0][i], maxInd[1][i], r[i]), file=out_file)
        diction[(maxInd[0][i], maxInd[1][i])] = r[i]
    #print(diction[10686, 1590])
    #for j in cand_index:
        #print(j)
        
def printOriginElev(elev_arr, out_file):
    num_rows = elev_arr.shape[0]
    num_cols = elev_arr.shape[1]
    for i in range(0, num_rows):
        for j in range(0, num_cols):
            print(elev_arr[i,j], end=' ', file=out_file)
        print("", file=out_file)

def main():
    print("For: {}, extracted elevation data in: {}, and local maxima data in: {}".format(sys.argv[1], sys.argv[2], sys.argv[3]))
    extr_data = open(sys.argv[2], "w")
    local_max = open(sys.argv[3], "w")
    dataset = gdal.Open(sys.argv[1], gdal.GA_ReadOnly)
    if not dataset:
        print("ERROR: CANNOT OPEN DEM FILE")
    elif not extr_data:
        print("ERROR: CANNOT OPEN EXTRACT DATA OUTPUT FILE")
    elif not local_max:
        print("ERROR: CANNOT OPEN LOCAL MAXIMA DATA OUTPUT FILE")
    else:
        geotransform = dataset.GetGeoTransform()

        if geotransform:
            # lets you know where the origin starts if viewed in qgis
            #print("Origin = {}, {}".format(geotransform[0], geotransform[3]))
            
            # Scale in meters (for our .tif files at least) for each pixel
            #print("Pixel Size = ({}, {})".format(geotransform[1], geotransform[5]))
            
            num_cols = dataset.RasterXSize
            num_rows = dataset.RasterYSize
            ext = testGdal.get_corner_coordinates(geotransform, num_cols, num_rows)
            src_spa_ref_sys = osr.SpatialReference()
            src_spa_ref_sys.ImportFromWkt(dataset.GetProjection())
            targ_spa_ref_sys = src_spa_ref_sys.CloneGeogCS()

            geo_ext = testGdal.reproj_coordinates(ext, src_spa_ref_sys, targ_spa_ref_sys)
            
            print("Coordinates of each corner pixel in degree decimal:", file=extr_data)
            # Gives corner coordinates in degree decimal (not degree minutes seconds etc)
            print(geo_ext, file=extr_data)

        # loads up a channel of the image i.e. r from rgb
        band = dataset.GetRasterBand(1)
        # The form in which the data is stored
        #print("Band Type={}".format(gdal.GetDataTypeName(band.DataType)))

        min = band.GetMinimum()
        max = band.GetMaximum()
  
        # if a max and min are not embedded in file
        if not min or not max:
            (min, max) = band.ComputeRasterMinMax(True)

        #print("Min={:.3f}, Max={:.3f}".format(min, max))

        # confirms that a channel is the size of the image but not necessarily all the info
        #print("Band size is {} x {}".format(band.XSize, band.YSize))

        # stores channel info as a large array, making it a np array allows for functions
        # and modules that come with numpy, we'll need it for sampling and stuff
        # n^2
        rasterArray = np.array(band.ReadAsArray())
        
        # obtains the value assigned in the file to rep nan
        nodata = band.GetNoDataValue()

        # filters out the nan values from the array likely n^2
        rasterArray = np.ma.masked_equal(rasterArray, nodata)
        
        # checking that the dimensions of the array are correct
        print(rasterArray.shape, file=extr_data)
        
        printOriginElev(rasterArray, extr_data)

        # freeing up data just in case
        band = None
        dataset = None
        #print("Elevation data")
        # both n^2 but checking if filtering out nan did anything
        #print("Min is: {} Max is: {}".format(np.min(rasterArray), np.max(rasterArray)))
        #for elev_data in np.nditer(rasterArray.T):
            #print(elev_data, end=' ')
        localMaxima(rasterArray, 100, local_max)

if __name__ == "__main__":
    main()


            
