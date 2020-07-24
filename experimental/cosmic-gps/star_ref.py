#!/usr/bin/env python
# star_ref.py


class Star_Ref:
    def __init__(self, sid, name, mag, ra, dec):
        self.sid = sid
        self.star_name = name
        self.star_mag = mag
        self.star_ra = ra
        self.star_dec = dec
        self.adj_list = dict()

    def add(self, star_id, angular_distance):
        self.adj_list[star_id] = angular_distance

    def get_sid(self):
        return self.sid

    def get_name(self):
        return self.star_name

    def get_mag(self):
        return self.star_mag

    def get_ra(self):
        return self.star_ra

    def get_dec(self):
        return self.star_dec

    def get_adj_list(self):
        return self.adj_list

    def print_star(self):
        print(
            self.sid,
            self.star_name,
            self.star_mag,
            self.star_ra,
            self.star_dec,
            self.adj_list,
        )
