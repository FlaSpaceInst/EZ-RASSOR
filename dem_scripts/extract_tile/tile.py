#!/usr/bin/env python
import os, gdal, sys, ntpath


def main():
    input_filename = sys.argv[1]
    input_desire_size = int(sys.argv[2])

    # Creates the beginning of output file name with the input filename without
    # the extension part
    output_filename = os.path.splitext(ntpath.basename(sys.argv[1]))[0] + "_tile_"

    ds = gdal.Open(input_filename)
    band = ds.GetRasterBand(1)
    xsize = band.XSize
    ysize = band.YSize

    # Moves a kernel across the DEM to create smaller DEM tiles
    for i in range(0, xsize, input_desire_size):
        for j in range(0, ysize, input_desire_size):
            com_string = (
                "gdal_translate -of GTIFF -srcwin "
                + str(i)
                + ", "
                + str(j)
                + ", "
                + str(input_desire_size)
                + ", "
                + str(input_desire_size)
                + " "
                + str(input_filename)
                + " "
                + "/tmp/results/"
                + str(output_filename)
                + str(i)
                + "_"
                + str(j)
                + ".tif"
            )
            # Executes the command
            os.system(com_string)


if __name__ == "__main__":
    main()
