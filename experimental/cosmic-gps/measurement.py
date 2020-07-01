#!/usr/bin/env python
# measurements.py

from math import radians
from math import degrees
from math import sin
from math import cos
from math import acos


def ra_to_decimal_deg(ra_h, ra_m, ra_s):
    return (float(ra_h) + (float(ra_m) / 60) + (float(ra_s) /3600)) * 15

def dec_to_decimal_deg(dec_d, dec_m, dec_s):
    if dec_d > 0:
        return float(dec_d) + (float(dec_m) / 60) + (float(dec_s) /3600)
    else:
        return float(dec_d) - (float(dec_m) / 60) - (float(dec_s) /3600)

def calculate_angular_distance(starA_ra, starA_dec, starB_ra, starB_dec):
    return degrees(
               acos(
                    (sin(radians(starA_dec)) *
                     sin(radians(starB_dec))
                    ) +
                    (cos(radians(starA_dec)) *
                     cos(radians(starB_dec)) *
                     cos(radians(starB_ra) - radians(starA_ra))
                    )
                   )
                  )

# used for position derivation takes three stars and distances to a shared
# point and returns the coor, can be used with celestial or glocal positions
def determine_coordinate(lamb1, phi1, d1, lamb2, phi2, d2,
                         lamb3, phi3, d3, search_angle, epsilon):
    epsilon = 0.000000001
    # take the smallest set to current
    if d3 > d2:
        cur_d = d2
        cur_lamb = lamb2
        cur_phi = phi2
    else:
        cur_d = d3
        cur_lamb = lamb3
        cur_phi = phi3
    if cur_d > d1:
        cur_d = d1
        cur_lamb = lamb1
        cur_phi = phi1
    # determine if the circle is polar
    polarFlag = False
    phi_north = cur_phi + cur_d
    if phi_north > 90.0:
        polarFlag = True
    phi_south = cur_phi - cur_d
    if phi_south < -90.0:
        polarFlag = True

    if polarFlag:
        cp = scan_polar(cur_lamb, cur_phi, cur_d,
                        lamb1, phi1, d1,
                        lamb2, phi2, d2,
                        lamb3, phi3, d3, epsilon)
    else:
        cp = scan_non_polar(cur_lamb, cur_phi, cur_d,
                            lamb1, phi1, d1,
                            lamb2, phi2, d2,
                            lamb3, phi3, d3, epsilon)
    return cp


def scan_polar(cur_lamb, cur_phi, d,
               lamb1, phi1, d1, lamb2, phi2, d2, lamb3, phi3, d3, epsilon):
    # if polar scan the equator starting at 0.0
    increment = 0.1
    temp_lamb = 0.0
    p_ratio = 9999.0
    min_lamb = 0.0
    min_phi = 0.0
    while temp_lamb < 360.0:
        find_phi = 0.0
        search_angle = 90.0
        dc = calculate_angular_distance(temp_lamb, find_phi,
                                        cur_lamb, cur_phi)
        c = abs(1 - (dc/d))
        while search_angle > epsilon:
            # Calculate distances at the search points
            phi_north = find_phi + search_angle
            phi_south = find_phi - search_angle
            dn = calculate_angular_distance(temp_lamb, phi_north,
                                            cur_lamb, cur_phi)
            n = abs(1 - (dn/d))
            ds = calculate_angular_distance(temp_lamb, phi_south,
                                            cur_lamb, cur_phi)
            s = abs(1 - (ds/d))
            # find the closest to 0.0 of n,e,s,w
            if s > n:
                t = n
                temp_phi = phi_north
            else:
                t = s
                temp_phi = phi_south
            if t > c:
                t = c
                temp_phi = find_phi
                search_angle = search_angle / 2.0
            c = t
            find_phi = temp_phi
        # record the pRatio with all three points
        d1p = calculate_angular_distance(temp_lamb, find_phi, lamb1, phi1)
        d2p = calculate_angular_distance(temp_lamb, find_phi, lamb2, phi2)
        d3p = calculate_angular_distance(temp_lamb, find_phi, lamb3, phi3)
        p = abs(1 - (d1p/d1)) + abs(1 - (d2p/d2)) + abs(1 - (d3p/d3))
        if p_ratio > p:
            p_ratio = p
            min_lamb = temp_lamb
            min_phi = find_phi
        temp_lamb += increment
    return (min_lamb, min_phi)


def scan_non_polar(cur_lamb, cur_phi, d,
                   lamb1, phi1, d1,
                   lamb2, phi2, d2,
                   lamb3, phi3, d3, epsilon):
    # if not polar scan the equator starting west
    big_increment = 0.1
    small_increment = 0.01

    offset = 0.0

    lamb_east = cur_lamb + d
    lamb_west = cur_lamb - d
    if lamb_east > 360.0:
        left_over = lamb_east - 360.0
        offset = -60.0 - left_over
        lamb_east = lamb_east + offset
        cur_lamb = cur_lamb + offset
        lamb_west = lamb_west + offset
        lamb1 = lamb1 + offset
        lamb2 = lamb2 + offset
        lamb3 = lamb3 + offset
    if lamb_west < 0.0:
        left_over = abs(lamb_west)
        offset = 60.0 + left_over
        lamb_east = lamb_east + offset
        cur_lamb = cur_lamb + offset
        lamb_west = lamb_west + offset
        lamb1 = lamb1 + offset
        lamb2 = lamb2 + offset
        lamb3 = lamb3 + offset
    p_ratio = 9999.0
    min_lamb = 0.0
    min_phi = 0.0

    # Left to Right / Approach from center to top
    temp_lamb = lamb_west
    while temp_lamb < lamb_east:
        find_phi = cur_phi
        search_angle = small_increment
        dc = calculate_angular_distance(temp_lamb, find_phi, cur_lamb, cur_phi)
        c = abs(1 - (dc/d))
        while d > search_angle:
            # Calculate distances at the search points
            phi_north = cur_phi + search_angle
            dn = calculate_angular_distance(temp_lamb, phi_north,
                                            cur_lamb, cur_phi)
            n = abs(1 - (dn/d))
            # find the closest to 0.0 of n
            if c > n:
                t = n
                temp_phi = phi_north
            else:
                t = c
                temp_phi = find_phi
            search_angle = search_angle + small_increment
            c = t
            find_phi = temp_phi
        # record the pRatio with all three points
        d1p = calculate_angular_distance(temp_lamb, find_phi, lamb1, phi1)
        d2p = calculate_angular_distance(temp_lamb, find_phi, lamb2, phi2)
        d3p = calculate_angular_distance(temp_lamb, find_phi, lamb3, phi3)
        p = abs(1 - (d1p/d1)) + abs(1 - (d2p/d2)) + abs(1 - (d3p/d3))
        if p_ratio > p:
            p_ratio = p
            min_lamb = temp_lamb
            min_phi = find_phi
        temp_lamb += big_increment
    
    # Right to Left / Approach from center to bottom
    temp_lamb = lamb_east
    while temp_lamb > lamb_west:
        find_phi = cur_phi
        search_angle = small_increment
        dc = calculate_angular_distance(temp_lamb, find_phi, cur_lamb, cur_phi)
        c = abs(1 - (dc/d))
        while d > search_angle:
            # Calculate distances at the search points
            phi_south = cur_phi - search_angle
            ds = calculate_angular_distance(temp_lamb, phi_south,
                                            cur_lamb, cur_phi)
            s = abs(1 - (ds/d))
            # find the closest to 0.0 of s
            if c > s:
                t = s
                temp_phi = phi_south
            else:
                t = c
                temp_phi = find_phi
            search_angle = search_angle + small_increment
            c = t
            find_phi = temp_phi
        # record the pRatio with all three points
        d1p = calculate_angular_distance(temp_lamb, find_phi, lamb1, phi1)
        d2p = calculate_angular_distance(temp_lamb, find_phi, lamb2, phi2)
        d3p = calculate_angular_distance(temp_lamb, find_phi, lamb3, phi3)
        p = abs(1 - (d1p/d1)) + abs(1 - (d2p/d2)) + abs(1 - (d3p/d3))
        if p_ratio > p:
            p_ratio = p
            min_lamb = temp_lamb
            min_phi = find_phi
        temp_lamb -= big_increment

    phi_north = cur_phi + d  # Will not be greater than 90.0
    phi_south = cur_phi - d  # Will not be less than -90.0
    
    # Bottom to Top / Approach from center to left
    temp_phi = phi_south
    while temp_phi < phi_north:
        find_lamb = cur_lamb
        search_angle = small_increment
        dc = calculate_angular_distance(find_lamb, temp_phi, cur_lamb, cur_phi)
        c = abs(1 - (dc/d))
        while (2*d) > search_angle:
            # Calculate distances at the search points
            lamb_west = cur_lamb - search_angle
            dw = calculate_angular_distance(lamb_west, temp_phi,
                                            cur_lamb, cur_phi)
            w = abs(1 - (dw/d))
            # find the closest to 0.0 of w
            if c > w:
                t = w
                temp_lamb = lamb_west
            else:
                t = c
                temp_lamb = find_lamb
            search_angle = search_angle + small_increment
            c = t
            find_lamb = temp_lamb
        # record the pRatio with all three points
        d1p = calculate_angular_distance(find_lamb, temp_phi, lamb1, phi1)
        d2p = calculate_angular_distance(find_lamb, temp_phi, lamb2, phi2)
        d3p = calculate_angular_distance(find_lamb, temp_phi, lamb3, phi3)
        p = abs(1 - (d1p/d1)) + abs(1 - (d2p/d2)) + abs(1 - (d3p/d3))
        if p_ratio > p:
            p_ratio = p
            min_lamb = find_lamb
            min_phi = temp_phi
        temp_phi += big_increment
    
    # Top to Bottom / Approach from center to right
    temp_phi = phi_north
    while temp_phi > phi_south:
        find_lamb = cur_lamb
        search_angle = small_increment
        dc = calculate_angular_distance(find_lamb, temp_phi, cur_lamb, cur_phi)
        c = abs(1 - (dc/d))
        while (2*d) > search_angle:
            # Calculate distances at the search points
            lamb_east = cur_lamb + search_angle
            de = calculate_angular_distance(lamb_east, temp_phi,
                                            cur_lamb, cur_phi)
            e = abs(1 - (de/d))
            # find the closest to 0.0 of e
            if c > e:
                t = e
                temp_lamb = lamb_east
            else:
                t = c
                temp_lamb = find_lamb
            search_angle = search_angle + small_increment
            c = t
            find_lamb = temp_lamb
        # record the pRatio with all three points
        d1p = calculate_angular_distance(find_lamb, temp_phi, lamb1, phi1)
        d2p = calculate_angular_distance(find_lamb, temp_phi, lamb2, phi2)
        d3p = calculate_angular_distance(find_lamb, temp_phi, lamb3, phi3)
        p = abs(1 - (d1p/d1)) + abs(1 - (d2p/d2)) + abs(1 - (d3p/d3))
        if p_ratio > p:
            p_ratio = p
            min_lamb = find_lamb
            min_phi = temp_phi
        temp_phi -= big_increment
    
    min_lamb = min_lamb - offset
    if min_lamb > 360.0:
        min_lamb = min_lamb - 360.0
    if min_lamb < 0.0:
        min_lamb = 360.0 + min_lamb
    min_phi = min_phi - offset
    return (min_lamb, min_phi, p_ratio)

# converts the cp to gp
def calculate_geographic_position(coor, gha_aries):
    # clock_time
    # gha_aries
    sha_celestial_position = 360 - coor[0]
    gha_celestial_position = sha_celestial_position + gha_aries
    if gha_celestial_position > 360.0:
        gha_celestial_position -= 360.0
    if gha_celestial_position >= 180.0:
        longitude = 180.0 - (gha_celestial_position - 180.0)
    else:
        longitude = 0.0 - gha_celestial_position
    latitude = coor[1]
    return (longitude, latitude)

# argv added in the following order:
# x, x0, f(x0), x1, f(x1), ..., xn, f(xn)
class Calibration_Function:

    def __init__(self, var_list, resp_list):
        self.var_list = var_list
        self.resp_list = resp_list

    def neville_interpolation(self, query):
        # trick:
        if query < self.var_list[2]:
            return self.resp_list[2] * (query / self.var_list[2])

        num_coef = len(self.var_list)
        q_list1 = [0] * num_coef
        q_list2 = list(q_list1)
        num_coef -= 1
        for i in range(num_coef):
            q_list1[0] = self.resp_list[i]
            q_list2[0] = self.resp_list[i+1]
            for j in range(num_coef):
                if (j <= i):
                    q_list2[j+1] = (((query - self.var_list[(i+1)-(j+1)])
                                      * q_list2[j]) -
                                    ((query - self.var_list[i+1]) *
                                     q_list1[j])) / (self.var_list[i+1] -
                                                   self.var_list[(i+1)-(j+1)])
            q_list1 = list(q_list2)
        return q_list2[num_coef]
