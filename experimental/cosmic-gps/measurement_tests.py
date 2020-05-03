#!/usr/bin/env python
# measurement_tests.py

from measurement import Calibration_Function
from measurement import ra_to_decimal_deg
from measurement import dec_to_decimal_deg
from measurement import calculate_angular_distance
from measurement import determine_coordinate
from measurement import calculate_geographic_position

class StarTestContainer:
    def __init__(self, n, a, d, ra, dec):
        self.name = n
        self.angle = a
        self.distance = d
        self.ra = ra
        self.dec = dec

def main():

    # The raw data of angle corresponding to pixel distance from the center.
    time_list = [13.0, 14.0, 15.0, 16.0, 17.0, 18.0,
                 19.0, 20.0, 21.0, 22.0, 23.0, 24.0,
                 25.0, 26.0, 27.0, 28.0, 29.0, 30.0]
    actual_gha_aries = [5.631667, 20.67333, 35.715, 50.755, 65.79667,
                        80.838333, 95.878333, 110.92, 125.961667, 141.001667,
                        156.04333, 171.08333, 186.125, 201.1667, 216.20667,
                        231.248333, 246.29, 261.33]
    gha_aries_calibration = Calibration_Function(time_list, actual_gha_aries)

    # coordinate covertion and distance tests

    stars1 = []

    # Alpha Canis Minoris 1st brightest
    star = StarTestContainer('Alpha Canis Minoris',
                             122.405568693,
                             25.1172042972,
                             ra_to_decimal_deg(7, 39, 18.12),
                             dec_to_decimal_deg(5, 13, 29.95))
    stars1.append(star)

    # Beta Geminorum 2nd brightest
    star = StarTestContainer('Beta Geminorum',
                             56.8996735767,
                             6.70418748113,
                             ra_to_decimal_deg(7, 45, 18.95),
                             dec_to_decimal_deg(28, 1, 34.32))
    stars1.append(star)

    # Alpha Leonis 3rd brightest // Maybe angle and dist
    star = StarTestContainer('Alpha Leonis',
                             207.811973537,
                             30.9907648048,
                             ra_to_decimal_deg(10, 8, 22.31),
                             dec_to_decimal_deg(11, 58, 1.95))
    stars1.append(star)

    # Beta Aurigae 4th brightest // Maybe angle and dist
    star = StarTestContainer('Beta Aurigae',
                             13.6613254946,
                             31.3855516368,
                             ra_to_decimal_deg(5, 59, 31.72),
                             dec_to_decimal_deg(44, 56, 50.76))
    stars1.append(star)

    # Alpha Geminorum 5th brightest
    star = StarTestContainer('Alpha Geminorum',
                             31.0045400679,
                             9.44994245983,
                             ra_to_decimal_deg(7, 34, 35.86),
                             dec_to_decimal_deg(31, 53, 17.79))
    stars1.append(star)

    # Gamma Leonis 6th brightest // Maybe angle and dist
    star = StarTestContainer('Gamma Leonis',
                             224.120815397,
                             29.3912178846,
                             ra_to_decimal_deg(10, 19, 58.35),
                             dec_to_decimal_deg(19, 50, 29.35))
    stars1.append(star)

    # Theta Aurigae 7th brightest
    star = StarTestContainer('Theta Aurigae',
                             28.4994353196,
                             29.6438533899,
                             ra_to_decimal_deg(5, 59, 43.27),
                             dec_to_decimal_deg(37, 12, 45.30))
    stars1.append(star)

    # Beta Canis Minoris 8th brightest
    star = StarTestContainer('Beta Canis Minoris',
                             112.598947606,
                             23.5008826209,
                             ra_to_decimal_deg(7, 27, 9.04),
                             dec_to_decimal_deg(8, 17, 21.54))
    stars1.append(star)
    print ''


    # compare calculated distances

    while stars1:
        starA = stars1.pop()
        for starB in stars1:
            actual = calculate_angular_distance(starA.ra, starA.dec,
                                                starB.ra, starB.dec)
            calcul = calculate_angular_distance(starA.angle,
                                                90.0 - starA.distance,
                                                starB.angle,
                                                90.0 - starB.distance)
            print 'Calculated distances between ',starA.name,' and ',starB.name
            print 'Actual ', actual
            print 'Calcul ', calcul
            print 'diff', calcul - actual
    print ''
    
    # Position derivation for top 3
    coor = determine_coordinate(ra_to_decimal_deg(7, 39, 18.12), # ACM
                                dec_to_decimal_deg(5, 13, 29.95),
                                25.1172042972,
                                ra_to_decimal_deg(7, 45, 18.95), # BG
                                dec_to_decimal_deg(28, 1, 34.32),
                                6.70418748113,
                                ra_to_decimal_deg(10, 8, 22.31), # AL
                                dec_to_decimal_deg(11, 58, 1.95),
                                30.9907648048,
                                30.0, 0.01)
    print 'Derived celestial position, RA: ', coor[0], ' DEC: ', coor[1]
    print ''
    
    # Global Position Derivation for top 3
    clock_time = 26.2708333
    gha_aries = gha_aries_calibration.neville_interpolation(clock_time)
    print 'GHA Aries: ', gha_aries
    gp = calculate_geographic_position(coor, gha_aries)
    print 'Derived geographic position, Long: ', gp[0], ' Lat: ', gp[1]



if __name__ == "__main__":
    main()
