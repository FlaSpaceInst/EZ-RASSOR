#!/usr/bin/env python
# star_cat_tests

from star_cat import Star_Cat
from star import Star


def main():

    angle_size = 0.5  # keep angle size (0.5, 0.25 0.125, 0.0625) deg
    fov = 90  # max fov 180 deg
    scope_size = 30

    # initialize the star reference catalogue
    star_catalogue = Star_Cat(
        "star_catalogue_init.txt", angle_size, fov, scope_size
    )

    list_of_stars = []

    # Alpha Canis Minoris 1st brightest #8
    star = Star()
    star.quick_set_angles(122.405568693, 25.1172042972)
    list_of_stars.append([1, star])

    # Beta Geminorum 2nd brightest #17
    star = Star()
    star.quick_set_angles(56.8996735767, 6.70418748113)
    list_of_stars.append([2, star])

    # Alpha Leonis 3rd brightest #21
    star = Star()
    star.quick_set_angles(207.811973537, 30.9907648048)
    list_of_stars.append([3, star])

    # Beta Aurigae 4th brightest #43
    star = Star()
    star.quick_set_angles(13.6613254946, 31.3855516368)
    list_of_stars.append([4, star])

    # Alpha Geminorum 5th brightest #23
    star = Star()
    star.quick_set_angles(31.0045400679, 9.44994245983)
    list_of_stars.append([5, star])

    # Gamma Leonis 6th brightest // Maybe angle and dist
    star = Star()
    star.quick_set_angles(224.120815397, 29.3912178846)
    list_of_stars.append([6, star])

    """ Star Matching """

    count = 0
    list_of_stars_size = len(list_of_stars)
    list_of_possible_matches = []

    # loop five times enforced by count
    while count < 5:
        # break early if too few stars
        if list_of_stars_size < 5:
            break

        star1 = list_of_stars[count + 0][1]
        star2 = list_of_stars[count + 1][1]
        star3 = list_of_stars[count + 2][1]
        star4 = list_of_stars[count + 3][1]
        star5 = list_of_stars[count + 4][1]
        stars = (star1, star2, star3, star4, star5)

        # Try matching to the scope for the first try
        if count == 0:
            list_of_possible_matches = star_catalogue.match_scope(stars)
        else:
            list_of_possible_matches = star_catalogue.match_global(stars)

        break  # remove only test

        # if no candidates are returned
        # remove the count+0 star with the next brightest
        count += 1
        list_of_stars_size -= 1
        if len(list_of_possible_matches) > 0:
            break  # can be possibly many group of candidates.
    print(list_of_possible_matches)


if __name__ == "__main__":
    main()
