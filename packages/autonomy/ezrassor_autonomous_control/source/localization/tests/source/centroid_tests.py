# centroid_tests
# 12/18/2019

# input: list of stars
# result: list, subpixel values and intensities

# test: outputs the original stars
# test: outputs the centers an intensities



from cent import centroid
from star import Star


def main():
    slist = []
    # setup stars
    star = Star()
    star.add(((30, 12), 50))
    star.add(((31, 11), 69))
    star.add(((31, 12), 97))
    star.add(((31, 13), 52))
    star.add(((32, 12), 83))
    slist.append(star)
    star = Star()
    star.add(((22, 92), 36))
    star.add(((23, 92), 72))
    star.add(((23, 93), 56))
    star.add(((24, 92), 43))
    star.add(((24, 93), 38))
    slist.append(star)
    star = Star()
    star.add(((8, 59), 45))
    star.add(((8, 60), 89))
    star.add(((8, 61), 57))
    star.add(((9, 60), 76))
    star.add(((9, 61), 57))
    slist.append(star)
    star = Star()
    star.add(((6, 36), 76))
    star.add(((6, 37), 53))
    star.add(((7, 36), 45))
    star.add(((7, 37), 39))
    slist.append(star)
    star = Star()
    star.add(((20, 40), 75))
    star.add(((20, 41), 43))
    star.add(((21, 40), 54))
    star.add(((21, 41), 43))
    slist.append(star)
    star = Star()
    star.add(((32, 23), 28))
    slist.append(star)

    printSList(slist)

    clist = []
    for s in slist:
        center = centroid(s)
        intensity = s.getIntensity()
        clist.append((center, intensity))

    printCList(clist)


def printSList(slist):
    print '***Stars***'
    print ''
    for s in slist:
        s.show()
        print ''

def printCList(clist):
    print '***Centers***'
    print ''
    for c in clist:
        print '*****'
        print 'Center:'
        print c[0]
        print ''
        print 'Intensity:'
        print c[1]
        print '_____'


if __name__ == "__main__":
    main()
