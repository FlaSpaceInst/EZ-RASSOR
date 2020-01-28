# star_cat.py



class Star_Cat:

    def __init__(self, file):
        # self.init_build(file)
        self.init_init(file)

    def init_built(self, file):
        # start
        # read from file each line at a time
        # each line has info for a star
        # each line has info for an angle
        # id, name, mag, coor_deg, adhjacent star tuples
        # deg bucket, star id list
        # create star objects
        # end

    def match(self, stars): # stars is a tuple of 5 star objects order brightest to less
        # start
        # create five arrays of length star_refs for each star
        # calculate the angular distance between each five star
        # use angles list and calculated distances to fill the five arrays
        # take the candidates from the arrays and check them with the adjacency list
        # create list of all possible candidate identities
        # return a list of tuples of 5 star_ref objects or null order as before
        # end

    def init_init(self, file):
        fptr = open(file, 'r')
        # each line has info for a star
        fptr.readline()
        # id, name, mag, coor
        # convert coor into deg
        # create adjacency list for each star
        # create star objects
        # create angles list
        # end

    def build(self):
        # start
        # write star objects into file
        # write adjacency list into file
        # write angle list into file
        # file name star_catalogue_built.txt
        # end

    def show_map(self):
        # start
        # write star coor on an image 1440 by 720 pixels.
        # return image
        # end

    def write_stat(self):
        # not sure yet

