# cent

# This module holds diffenent functions for centroiding a star
# input: star object
# outupt: subpixel center

# *********************#
# method not optimized #
# *********************#


from star import Star


def centroid(star):
    pDict = star.getDict()
    minRow = star.getMinRow()
    maxRow = star.getMaxRow()
    minCol = star.getMinCol()
    maxCol = star.getMaxCol()
    intensity = star.getIntensity()
    # get marginal distribution of rows
    vCols = []
    col = minCol
    while col <= maxCol:
        vI = 0
        row = minRow
        while row <= maxRow:
            if (row, col) in pDict:
                vI = vI + pDict[(row, col)]
            row = row + 1
        vCols.append(vI)
        col = col + 1
    # get marginal distribution of columns
    vRows = []
    row = minRow
    while row <= maxRow:
        vJ = 0
        col = minCol
        while col <= maxCol:
            if (row, col) in pDict:
                vJ = vJ + pDict[(row, col)]
            col = col + 1
        vRows.append(vJ)
        row = row + 1
    # get the mean of I
    tmp = 0
    c = 0.5
    for x in vCols:
        tmp = tmp + (c * x)
        c = c + 1
    meanI = tmp / intensity
    # get the mean of J
    tmp = 0
    c = 0.5
    for x in vRows:
        tmp = tmp + (c * x)
        c = c + 1
    meanJ = tmp / intensity
    centerRow = meanJ + minRow
    centerCol = meanI + minCol 
    return (centerRow, centerCol)

