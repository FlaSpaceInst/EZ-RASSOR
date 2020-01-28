# star.py


class Star:

    def __init__(self):  # constructor
        self.pdict = dict()
        self.intensity = 0
        self.minRow = 99999
        self.minCol = 99999
        self.maxRow = 0
        self.maxCol = 0

    def add(self, px):   # adds pixel
        row = px[0][0]
        col = px[0][1]
        intensity = px[1]
        self.pdict[(row, col)] = intensity
        self.intensity = self.intensity + intensity
        if row < self.minRow:
            self.minRow = row
        if row > self.maxRow:
            self.maxRow = row
        if col < self.minCol:
            self.minCol = col
        if col > self.maxCol:
            self.maxCol = col

    def getDict(self):
        return self.pdict

    def getMinRow(self):
        return self.minRow

    def getMaxRow(self):
        return self.maxRow

    def getMinCol(self):
        return self.minCol

    def getMaxCol(self):
        return self.maxCol

    def getIntensity(self):
        return self.intensity

    def shape(self): # returns the shape of the star
        rows = self.maxRow - self.minRow + 1
        cols = self.maxCol - self.minCol + 1
        if rows > cols:
            return float(cols) / rows
        else:
            return float(rows) / cols

    def show(self):  # shows the pixel formated
        print '*****'
        print 'Pixels:'
        print self.pdict
        print ''
        print 'Intensity:'
        print self.intensity
        print ''
        print 'Shape:'
        print self.shape()
        print '_____'

