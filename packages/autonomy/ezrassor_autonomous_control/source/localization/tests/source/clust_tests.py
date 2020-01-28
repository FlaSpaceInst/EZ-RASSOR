# clust_tests
# 12/17/2019

# input: dictionary with pixel coordinates and intensities
# result: list, clusted pixels (stars)

# test: outputs the original dictionary
# test: outputs the star clusters


from clust import cluster
from star import Star


def main():
    # get test dictionary
    pixls = {
        (30, 12): 50,
        (24, 92): 43,
        (23, 93): 56,
        (31, 13): 52,
        (8, 59): 45,
        (6, 36): 76,
        (24, 93): 38,
        (9, 61): 57,
        (23, 92): 72,
        (31, 12): 97,
        (21, 41): 43,
        (7, 37): 39,
        (20, 40): 75,
        (6, 37): 53,
        (31, 11): 69,
        (8, 60): 89,
        (9, 60): 76,
        (7, 36): 45,
        (8, 61): 57,
        (21, 40): 54,
        (20, 41): 43,
        (32, 12): 83,
        (22, 92): 36,
        (32, 23): 28
    }

    slist = cluster(pixls)
    printSList(slist)


def printSList(slist):
    print '***Stars***'
    print ''
    # loop through list
    for s in slist:
        s.show()
        print ''


if __name__ == "__main__":
    main()

