# star_ref.py



class Star_Ref:

    def __init__(self, name, mag, ra, dec):
        self.starName = name
        self.starMag = mag
        self.starRA = ra
        self.starDEC = dec

    def getName(self):
        return self.starName

    def getMag(self):
        return self.starMag

    def getRA(self):
        return self.starRA

    def getDEC(self):
        return self.starDEC

