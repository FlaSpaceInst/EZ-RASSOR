#!/usr/bin/env python
# star_cat.py

from star_ref import Star_Ref
from measurement import ra_to_decimal_deg
from measurement import dec_to_decimal_deg
from measurement import calculate_angular_distance


class Star_Cat:
    def __init__(self, image_file, angle_size, fov, scope_size):
        init_ra_position = 0.0
        init_dec_position = 0.0
        hfov = float(fov) / 2
        num_angles = int(round(fov / angle_size)) + 1

        fptr = open(image_file, "r")
        fptr.readline()  # first line unimportant
        self.adj_star_list = dict()
        # each line has info for a star
        count_global = 0
        while True:
            star_info = fptr.readline()
            if star_info == "":
                break
            star_splits = star_info.split(" ")[:-1]
            # Convert RA to decimal
            ra = ra_to_decimal_deg(
                float(star_splits[3]),
                float(star_splits[4]),
                float(star_splits[5]),
            )
            # Covert DEC to decimal
            dec = dec_to_decimal_deg(
                float(star_splits[6]),
                float(star_splits[7]),
                float(star_splits[8]),
            )
            # Create star_ref
            self.adj_star_list[int(star_splits[0])] = Star_Ref(
                int(star_splits[0]),
                star_splits[1],
                int(star_splits[2]),
                ra,
                dec,
            )
            count_global += 1
        fptr.close()
        # create angles list
        # complete adjacency list for each star
        self.angles_list = dict()
        for x in range(num_angles):
            self.angles_list[x] = set()
        for sid_starA in self.adj_star_list:
            for sid_starB in self.adj_star_list:
                if sid_starA == sid_starB:
                    continue
                else:
                    # measure angular distance between starA and StarB
                    angular_distance = calculate_angular_distance(
                        self.adj_star_list[sid_starA].get_ra(),
                        self.adj_star_list[sid_starA].get_dec(),
                        self.adj_star_list[sid_starB].get_ra(),
                        self.adj_star_list[sid_starB].get_dec(),
                    )
                    if angular_distance < fov:
                        self.adj_star_list[sid_starA].add(
                            sid_starB, angular_distance
                        )
                        elem = int(angular_distance * (1 / angle_size))
                        if elem == 0:
                            self.angles_list[elem].update(
                                [sid_starA, sid_starB]
                            )
                            self.angles_list[elem + 1].update(
                                [sid_starA, sid_starB]
                            )
                        else:
                            self.angles_list[elem - 1].update(
                                [sid_starA, sid_starB]
                            )
                            self.angles_list[elem].update(
                                [sid_starA, sid_starB]
                            )
                            self.angles_list[elem + 1].update(
                                [sid_starA, sid_starB]
                            )
        # Create scopeAdjStarList
        count_scope = 0
        self.scope_adj_star_list = dict()
        for sid_star in self.adj_star_list:
            ra = self.adj_star_list[sid_star].get_ra()
            dec = self.adj_star_list[sid_star].get_dec()
            angular_distance = calculate_angular_distance(
                ra, dec, init_ra_position, init_dec_position
            )
            if angular_distance < (hfov + scope_size):
                self.scope_adj_star_list[sid_star] = Star_Ref(
                    sid_star,
                    self.adj_star_list[sid_star].get_name(),
                    self.adj_star_list[sid_star].get_mag(),
                    ra,
                    dec,
                )
                count_scope += 1
        # create scope angles list
        # complete scope adjacency list for each star
        self.scope_angles_list = dict()
        for x in range(num_angles):
            self.scope_angles_list[x] = set()
        for sid_starA in self.scope_adj_star_list:
            for sid_starB in self.scope_adj_star_list:
                if sid_starA == sid_starB:
                    continue
                else:
                    # measure angular distance between starA and StarB
                    angular_distance = calculate_angular_distance(
                        self.scope_adj_star_list[sid_starA].get_ra(),
                        self.scope_adj_star_list[sid_starA].get_dec(),
                        self.scope_adj_star_list[sid_starB].get_ra(),
                        self.scope_adj_star_list[sid_starB].get_dec(),
                    )
                    if angular_distance < fov:
                        self.scope_adj_star_list[sid_starA].add(
                            sid_starB, angular_distance
                        )
                        elem = int(angular_distance * (1 / angle_size))
                        if elem == 0:
                            self.scope_angles_list[elem].update(
                                [sid_starA, sid_starB]
                            )
                            self.scope_angles_list[elem + 1].update(
                                [sid_starA, sid_starB]
                            )
                        else:
                            self.scope_angles_list[elem - 1].update(
                                [sid_starA, sid_starB]
                            )
                            self.scope_angles_list[elem].update(
                                [sid_starA, sid_starB]
                            )
                            self.scope_angles_list[elem + 1].update(
                                [sid_starA, sid_starB]
                            )
        self.angle_size = angle_size
        self.hfov = hfov
        self.fov = fov
        self.num_angles = num_angles
        self.num_stars = count_global
        self.scope_size = scope_size
        self.scope_position_ra = init_ra_position
        self.scope_position_dec = init_dec_position
        self.scope_num_stars = count_scope
        # end

    def rescope(self, ra_position, dec_position):  # in degrees
        angular_distance = calculate_angular_distance(
            ra_position,
            dec_position,
            self.scope_position_ra,
            self.scope_position_dec,
        )
        if angular_distance < self.scope_size:
            return False
        # Create scope_adj_star_list
        count_scope = 0
        self.scope_adj_star_list = dict()
        for sid_star in self.adj_star_list:
            ra = self.adj_star_list[sid_star].get_ra()
            dec = self.adj_star_list[sid_star].get_dec()
            angular_distance = calculate_angular_distance(
                ra, dec, ra_position, dec_position
            )
            if angular_distance < (self.hfov + self.scope_size):
                self.scope_adj_star_list[sid_star] = Star_Ref(
                    sid_star,
                    self.adj_star_list[sid_star].get_name(),
                    self.adj_star_list[sid_star].get_mag(),
                    ra,
                    dec,
                )
                count_scope += 1
        # create scope angles list
        # complete scope adjacency list for each star
        self.scope_angles_list = dict()
        for x in range(self.num_angles):
            self.scope_angles_list[x] = set()
        for sid_starA in self.scope_adj_star_list:
            for sid_starB in self.scope_adj_star_list:
                if sid_starA == sid_starB:
                    continue
                else:
                    # measure angular distance between starA and StarB
                    angular_distance = calculate_angular_distance(
                        self.scope_adj_star_list[sid_starA].get_ra(),
                        self.scope_adj_star_list[sid_starA].get_dec(),
                        self.scope_adj_star_list[sid_starB].get_ra(),
                        self.scope_adj_star_list[sid_starB].get_dec(),
                    )
                    if angular_distance < self.fov:
                        self.scope_adj_star_list[sid_starA].add(
                            sid_starB, angular_distance
                        )
                        elem = int(angular_distance * (1 / self.angle_size))
                        if elem == 0:
                            self.scope_angles_list[elem].update(
                                [sid_starA, sid_starB]
                            )
                            self.scope_angles_list[elem + 1].update(
                                [sid_starA, sid_starB]
                            )
                        else:
                            self.scope_angles_list[elem - 1].update(
                                [sid_starA, sid_starB]
                            )
                            self.scope_angles_list[elem].update(
                                [sid_starA, sid_starB]
                            )
                            self.scope_angles_list[elem + 1].update(
                                [sid_starA, sid_starB]
                            )
        self.scope_position_ra = ra_position
        self.scope_position_dec = dec_position
        self.scope_num_stars = count_scope
        return True

    def match_global(self, stars):  # 5 stars ordered brightest to least
        return match(
            stars,
            self.angle_size,
            self.num_stars,
            self.angle_size,
            self.angles_list,
            self.adj_star_list,
        )
        # end

    def match_scope(self, stars):  # 5 star objects order brightest to less
        list_of_possible_matches = match(
            stars,
            self.angle_size,
            self.num_stars,
            self.angle_size,
            self.angles_list,
            self.scope_adj_star_list,
        )
        # if the scope fails possibly invalid, try global.
        if len(list_of_possible_matches) == 0:
            list_of_possible_matches = self.match_global(stars)
        return list_of_possible_matches
        # end

    # test function
    def print_adj_star_list(self):
        for sid_star in self.adj_star_list:
            self.adj_star_list[sid_star].print_star()

    # test function
    def print_scope_adj_star_list(self):
        for sid_star in self.scope_adj_star_list:
            self.scope_adj_star_list[sid_star].print_star()

    # test function, params are local
    def print_distribution(self):
        # scan all stars and fill the buckets
        for sid_star in self.adj_star_list:
            ra = self.adj_star_list[sid_star].get_ra()
            dec = self.adj_star_list[sid_star].get_dec()
            print("For quad ra", ra, "dec", dec)

    # test function
    def print_angles(self):
        # How many stars have each angle
        for x in range(self.num_angles):
            size = x * self.angle_size
            print("For angle ", size, self.angles_list.get(x))

    # test function
    def print_scope_angles(self):
        # How many stars have each angle
        for x in range(self.num_angles):
            size = x * self.angle_size
            print("For angle ", size, self.scope_angles_list.get(x))


def match(stars, epsilon, num_stars, angle_size, angles_list, adj_star_list):
    # start
    match_size = 5  # don't change
    star_array = dict()
    # create five arrays of length star_refs for each star
    for x in range(match_size):
        star_array[x] = dict()
        for starID in range(num_stars):
            star_array[x][starID] = 0
    # calculate the angular distance between each five star
    angular_distances = dict()
    for x in range(match_size):
        for y in range(match_size):
            # calculate distances for each pair
            if x < y:
                calculated = calculate_angular_distance(
                    stars[x].get_direction_angle(),
                    90.0 - stars[x].get_distance_angle(),
                    stars[y].get_direction_angle(),
                    90.0 - stars[y].get_distance_angle(),
                )
                angular_distances[(x, y)] = calculated
    # use angles list and calculated distances to fill the five arrays
    for x in range(match_size):
        for y in range(match_size):
            if x < y:
                elem = int(angular_distances[(x, y)] * (1 / angle_size))
                for z in angles_list[elem]:
                    star_array[x][z] += 1
                    star_array[y][z] += 1
    # take the candidates from the arrays
    candidates = dict()
    for x in range(match_size):
        candidates[x] = []
        count = 0
        for starID in range(num_stars):
            if starID in adj_star_list:  # included
                if star_array[x][starID] == (match_size - 1):
                    count += 1
                    candidates[x].append(starID)
        if count == 0:
            return []  # Not good match failed.
    # create permutations
    # check them with the adjacency list
    # create list of all possible candidate identities
    candidate_temp_temp_permutations = []
    for x in candidates[0]:
        candidate_temp_temp_permutations.append((x))
    candidate_temp_permutations = []
    candidate_permutations = []
    for combo in candidate_temp_temp_permutations:
        for x in candidates[1]:
            if combo != x:
                candidate_temp_permutations.append((combo, x))
    for combo in candidate_temp_permutations:
        distance01 = calculate_angular_distance(
            adj_star_list[combo[0]].get_ra(),
            adj_star_list[combo[0]].get_dec(),
            adj_star_list[combo[1]].get_ra(),
            adj_star_list[combo[1]].get_dec(),
        )
        if distance01 < (angular_distances[(0, 1)] + epsilon) and distance01 > (
            angular_distances[(0, 1)] - epsilon
        ):
            candidate_permutations.append(combo)
    if not candidate_permutations:
        return candidate_permutations
    candidate_temp_temp_permutations = candidate_permutations
    candidate_temp_permutations = []
    candidate_permutations = []
    for combo in candidate_temp_temp_permutations:
        for x in candidates[2]:
            if combo[0] != x and combo[1] != x:
                candidate_temp_permutations.append((combo[0], combo[1], x))
    for combo in candidate_temp_permutations:
        distance02 = calculate_angular_distance(
            adj_star_list[combo[0]].get_ra(),
            adj_star_list[combo[0]].get_dec(),
            adj_star_list[combo[2]].get_ra(),
            adj_star_list[combo[2]].get_dec(),
        )
        distance12 = calculate_angular_distance(
            adj_star_list[combo[1]].get_ra(),
            adj_star_list[combo[1]].get_dec(),
            adj_star_list[combo[2]].get_ra(),
            adj_star_list[combo[2]].get_dec(),
        )
        if distance02 < (angular_distances[(0, 2)] + epsilon) and distance02 > (
            angular_distances[(0, 2)] - epsilon
        ):
            if distance12 < (
                angular_distances[(1, 2)] + epsilon
            ) and distance12 > (angular_distances[(1, 2)] - epsilon):
                candidate_permutations.append(combo)
    if not candidate_permutations:
        return candidate_permutations
    candidate_temp_temp_permutations = candidate_permutations
    candidate_temp_permutations = []
    candidate_permutations = []
    for combo in candidate_temp_temp_permutations:
        for x in candidates[3]:
            if combo[0] != x and combo[1] != x and combo[2] != x:
                candidate_temp_permutations.append(
                    (combo[0], combo[1], combo[2], x)
                )
    for combo in candidate_temp_permutations:
        distance03 = calculate_angular_distance(
            adj_star_list[combo[0]].get_ra(),
            adj_star_list[combo[0]].get_dec(),
            adj_star_list[combo[3]].get_ra(),
            adj_star_list[combo[3]].get_dec(),
        )
        distance13 = calculate_angular_distance(
            adj_star_list[combo[1]].get_ra(),
            adj_star_list[combo[1]].get_dec(),
            adj_star_list[combo[3]].get_ra(),
            adj_star_list[combo[3]].get_dec(),
        )
        distance23 = calculate_angular_distance(
            adj_star_list[combo[2]].get_ra(),
            adj_star_list[combo[2]].get_dec(),
            adj_star_list[combo[3]].get_ra(),
            adj_star_list[combo[3]].get_dec(),
        )
        if distance03 < (angular_distances[(0, 3)] + epsilon) and distance03 > (
            angular_distances[(0, 3)] - epsilon
        ):
            if distance13 < (
                angular_distances[(1, 3)] + epsilon
            ) and distance13 > (angular_distances[(1, 3)] - epsilon):
                if distance23 < (
                    angular_distances[(2, 3)] + epsilon
                ) and distance23 > (angular_distances[(2, 3)] - epsilon):
                    candidate_permutations.append(combo)
    if not candidate_permutations:
        return candidate_permutations
    candidate_temp_temp_permutations = candidate_permutations
    candidate_temp_permutations = []
    candidate_permutations = []
    for combo in candidate_temp_temp_permutations:
        for x in candidates[4]:
            if (
                combo[0] != x
                and combo[1] != x
                and combo[2] != x
                and combo[3] != x
            ):
                candidate_temp_permutations.append(
                    (combo[0], combo[1], combo[2], combo[3], x)
                )
    for combo in candidate_temp_permutations:
        distance04 = calculate_angular_distance(
            adj_star_list[combo[0]].get_ra(),
            adj_star_list[combo[0]].get_dec(),
            adj_star_list[combo[4]].get_ra(),
            adj_star_list[combo[4]].get_dec(),
        )
        distance14 = calculate_angular_distance(
            adj_star_list[combo[1]].get_ra(),
            adj_star_list[combo[1]].get_dec(),
            adj_star_list[combo[4]].get_ra(),
            adj_star_list[combo[4]].get_dec(),
        )
        distance24 = calculate_angular_distance(
            adj_star_list[combo[2]].get_ra(),
            adj_star_list[combo[2]].get_dec(),
            adj_star_list[combo[4]].get_ra(),
            adj_star_list[combo[4]].get_dec(),
        )
        distance34 = calculate_angular_distance(
            adj_star_list[combo[3]].get_ra(),
            adj_star_list[combo[3]].get_dec(),
            adj_star_list[combo[4]].get_ra(),
            adj_star_list[combo[4]].get_dec(),
        )
        if distance04 < (angular_distances[(0, 4)] + epsilon) and distance04 > (
            angular_distances[(0, 4)] - epsilon
        ):
            if distance14 < (
                angular_distances[(1, 4)] + epsilon
            ) and distance14 > (angular_distances[(1, 4)] - epsilon):
                if distance24 < (
                    angular_distances[(2, 4)] + epsilon
                ) and distance24 > (angular_distances[(2, 4)] - epsilon):
                    if distance34 < (
                        angular_distances[(3, 4)] + epsilon
                    ) and distance34 > (angular_distances[(3, 4)] - epsilon):
                        candidate_permutations.append(combo)
    if not candidate_permutations:
        return candidate_permutations
    candidate_temp_permutations = candidate_permutations
    candidate_permutations = []
    for combo in candidate_temp_permutations:
        candidate_permutations.append(
            (
                (
                    combo[0],
                    adj_star_list[combo[0]].get_name(),
                    adj_star_list[combo[0]].get_ra(),
                    adj_star_list[combo[0]].get_dec(),
                    stars[0].get_distance_angle(),
                ),
                (
                    combo[1],
                    adj_star_list[combo[1]].get_name(),
                    adj_star_list[combo[1]].get_ra(),
                    adj_star_list[combo[1]].get_dec(),
                    stars[1].get_distance_angle(),
                ),
                (
                    combo[2],
                    adj_star_list[combo[2]].get_name(),
                    adj_star_list[combo[2]].get_ra(),
                    adj_star_list[combo[2]].get_dec(),
                    stars[2].get_distance_angle(),
                ),
                (
                    combo[3],
                    adj_star_list[combo[3]].get_name(),
                    adj_star_list[combo[3]].get_ra(),
                    adj_star_list[combo[3]].get_dec(),
                    stars[3].get_distance_angle(),
                ),
                (
                    combo[4],
                    adj_star_list[combo[4]].get_name(),
                    adj_star_list[combo[4]].get_ra(),
                    adj_star_list[combo[4]].get_dec(),
                    stars[4].get_distance_angle(),
                ),
            )
        )
    return candidate_permutations  # 5 star tuples or null order as before
