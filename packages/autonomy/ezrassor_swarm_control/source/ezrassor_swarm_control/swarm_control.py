import rospy
import Queue
import numpy as np
import time
import math
from cv2 import imread
from threading import Thread, Lock
from geometry_msgs.msg import Point

from constants import commands

from ezrassor_swarm_control.msg import Path

from path_planner import PathPlanner
from swarm_utils import euclidean_distance
from swarm_utils import get_rover_status, preempt_rover_path
from swarm_utils import create_level_instructions
from swarm_utils import pixel_normalize

import os

from std_msgs.msg import Int8
from enum import Enum


class RoverProblemError(Exception):
    """Raised when rover encounters an issue."""

    pass


class ActionCodes(Enum):
    """Enumerate status codes which rover objects will use to differentiate which actions
    have been completed."""

    no_action_completed = 0
    path_completed = 1
    action_completed = 2
    preempted = 3


class RoverState(Enum):
    """Rover status which may indicate specific problems."""

    okay = 0
    infinite_pathing = 1
    cannot_move = 2


class Rover:
    """Abstract class for the controller's knowledge of an EZRASSOR."""

    # @staticmethod
    def create_command(self, cmd):
        path = Path()
        path.path.append(commands[cmd])
        return path

    def __init__(self, id_, rover_queue, lander_loc):
        # The id of the rover that this object controls.
        self.id_ = id_
        # The rover's current action.
        self.activity = "idle"

        # The rover's last known battery amount.
        # Must call get_rover_status_ to update.
        self.battery = None
        # The rover's last known battery amount.
        # Must call get_rover_status_ to update.
        self.position = None

        # Set to True to kill the rover thread.
        self.kill_thread = False
        # Thread that runs this rover object.
        self.thread = Thread(target=self.run_rover)

        # Location assigned by SwarmController to dig.
        self.dig_location = None
        # Location assigned by SwarmController to dump.
        self.dump_location = None
        # Location to charge.
        self.lander_loc = lander_loc

        # Reference of leveling instruction data structure.
        self.sub_pair_actions = None
        # Lock to prevent access.
        self.sub_pair_actions_lock = None

        # dig locations which have been assigned.
        self.dig_locations_assigned = None
        # dump locations which have been assigned.
        self.dump_locations_assigned = None
        # Lock to locations not assigned sets.
        self.locations_assigned_lock = None

        # Queue of rover ids which need an assignment from SwarmController.
        self.rover_queue = rover_queue
        # This rover needs an assignment at instaniation.
        self.rover_queue.put(self.id_)

        # Object to get pathing instructions.
        self.path_planner = None

        # Action client to communicate with backend.
        self.waypoint = rospy.Publisher(
            "/ezrassor{}/waypoint_client".format(self.id_), Path, queue_size=10
        )

        # Sub to get messages from backend.
        self.action_completed_sub = rospy.Subscriber(
            "/ezrassor{}/waypoint_client_action_completed".format(self.id_),
            Int8,
            self.set_action_completed,
        )
        # Encoded action status code indicating which action has been completed.
        self.action_completed = ActionCodes.no_action_completed.value

        # Battery calculation settings.
        # The cost of moving one waypoint.
        self.move_battery_cost = 0.1
        # The cost of moving one waypoint full of dirt.
        self.move_battery_cost_with_dirt = 0.2
        # The cost of completing one dig action.
        self.dig_battery_cost = 10.0
        # The cost of completing one dump action.
        self.dump_battery_cost = 12.0

        # Status code indicating spefific rover problems, if any.
        self.need_help = RoverState.okay.value

        # Indicates if the rover has dug since the last time it dumped.
        self.drum_full = False

        # Function to run when rover status is "executing custom routine"
        self.custom_routine = None

        # Save rover spawn point to go back to at the end of the program.
        self.home = None

    def get_rover_status_(self):
        """Request the SwarmController to update this rover's position and battery."""
        rover_status = get_rover_status(self.id_)
        self.position = rover_status.pose.position
        rover_status.battery -= 10
        self.battery = rover_status.battery
        return rover_status

    def preempt_rover_path_(self):
        """Request the SwarmController to preempt the rover's current pathing
        routine."""
        preempt_rover_path(self.id_)

    def set_action_completed(self, res):
        """Callback for when waypoint client tells the rover that the desired action has
        been completed."""
        res = res.data
        # Set rover status code.
        self.action_completed = res

    def dig(self):
        """Command the rover to dig at its current location."""
        # TODO: Add partial digs.
        rospy.loginfo("Rover {} digging.".format(self.id_))
        self.waypoint.publish(self.create_command("DIG"))
        self.activity = "digging"

        # Wait until digging is finished.
        while self.action_completed == ActionCodes.no_action_completed.value:
            rospy.sleep(1.0)

        # Reset rover action status code.
        self.action_completed = ActionCodes.no_action_completed.value
        self.activity = "idle"
        self.drum_full = True

    def dump(self):
        """Command the rover to dump at its current location."""
        rospy.loginfo("Rover {} dumping.".format(self.id_))

        self.waypoint.publish(self.create_command("DUMP"))
        self.activity = "dumping"

        # Wait until dumping is finished.
        while self.action_completed == ActionCodes.no_action_completed.value:
            rospy.sleep(1.0)

        # Reset rover action status code.
        self.action_completed = ActionCodes.no_action_completed.value
        self.activity = "idle"
        self.drum_full = False

    def charge(self):
        """Command the rover to charge at its current location."""
        self.waypoint.publish(self.create_command("CHG"))
        self.activity = "charging"

        # Wait until charging is finished.
        while self.action_completed == ActionCodes.no_action_completed.value:
            rospy.sleep(1.0)

        # Reset rover action status code.
        self.action_completed = ActionCodes.no_action_completed.value
        self.activity = "idle"

    def go_to_charge(self, path=None):
        """Command rover to complete charging routine."""
        # Go to charging site.
        self.go_to(self.home, "charging station", path)
        self.charge()

    def go_to(self, target_point, target_name, path=None):
        """Create pathing and command rover to follow it."""
        # If a path is not given, create one.
        if not path:
            path = self.path_planner.find_path(
                self.get_rover_status_().pose.position, target_point
            )

        if path:
            # Record start time to check for timeout.
            start_time = time.time()

            # Send path to be completed
            self.waypoint.publish(path)

            self.activity = "driving to {} at {} {}".format(
                target_name, target_point.x, target_point.y
            )

            rospy.loginfo(
                "Rover {} driving to {} at ({}, {})".format(
                    self.id_, target_name, target_point.x, target_point.y
                )
            )

            # Wait for waypoint to pushlish path completed code.
            while self.action_completed != ActionCodes.path_completed.value:
                # Check if pathing time has exceeded the timout time limit.
                if time.time() - start_time > 100:
                    rospy.loginfo(
                        "Rover {} exceeded 100 seconds".format(self.id_)
                    )
                    # Find if rover cannot move or if it cannot reach its destination.
                    # If the rover has moved since sleep began, it's in an infinite
                    # pathing loop.
                    self.get_rover_status_()
                    position_start = self.position
                    rospy.sleep(20.0)

                    # Check if rover has arrived at the destination during thread sleep.
                    if (
                        self.action_completed
                        == ActionCodes.path_completed.value
                    ):
                        rospy.loginfo(
                            "Rover {} reached destination during diagnoses".format(
                                self.id_
                            )
                        )
                        break

                    # Stop rover pathing.
                    rospy.loginfo("Preempting Rover {} path".format(self.id_))
                    self.preempt_rover_path_()

                    # Wait for rover to stop pathing.
                    while self.action_completed != ActionCodes.preempted.value:
                        rospy.sleep(1.0)
                    # Reset rover status code.
                    self.action_completed = (
                        ActionCodes.no_action_completed.value
                    )

                    # Diagnose infinite rover pathing or rover cannot move status.
                    self.get_rover_status_()
                    if (
                        abs(position_start.x - self.position.x) > 0.25
                        or abs(position_start.y - self.position.y) > 0.25
                    ):
                        self.need_help = RoverState.infinite_pathing.value
                    else:
                        self.need_help = RoverState.cannot_move.value
                    name = (
                        "infinite pathing"
                        if self.need_help == 1
                        else "cannot move"
                    )
                    rospy.loginfo(
                        "Rover {} diagnoses: {}".format(self.id_, name)
                    )

                    # Raise exception
                    rospy.loginfo("Rover {} aborting actions".format(self.id_))
                    raise RoverProblemError
                # QUESTION: Does this make this object sleep or rover sleep?
                rospy.sleep(1.0)

            # Reset rover status code.
            self.action_completed = ActionCodes.no_action_completed.value
            self.activity = "idle"

    def battery_check(
        self,
        actions,
        battery_cost_of_one_action,
        battery_before,
        expected_battery_difference,
        battery_from_dump_to_charge,
    ):
        """Recalculate the amount of actions possible by a rover given a possibly
        unexpected battery difference due to an action."""
        self.get_rover_status_()
        # Check if battery cost of action was more than expected without padding.
        if battery_before - self.battery > (expected_battery_difference * 0.9):
            # Calculate actual battery cost of action and add padding.
            battery_difference = (battery_before - self.battery) * 1.1
            # Recalculate  the cost of a single action.
            # Subtract the estimated cost of sub-action from action battery cost.
            battery_cost_of_one_action -= expected_battery_difference
            # Add the actual cost of sub-action from action battery cost.
            battery_cost_of_one_action += battery_difference
            # Calculate a better estimate of actions possible.
            actions = (
                self.battery - battery_from_dump_to_charge
            ) // battery_cost_of_one_action
            return actions, battery_difference, battery_cost_of_one_action
        return actions, expected_battery_difference, battery_cost_of_one_action

    def abort(self):
        # Queue for help from SwarmController.
        self.rover_queue.put(self.id_)
        rospy.loginfo("Rover {} queued.".format(self.id_))

    def level(self):
        """Dig at assigned dig location and dump at assigned dump location until area is level.
        Check at every step if the rover needs to go charge."""
        rospy.loginfo(
            "Rover {} is calculating battery requirements.".format(self.id_)
        )

        # Precalculate getting to dig site from current rover position.
        path_from_rover_to_dig = self.path_planner.find_path(
            self.get_rover_status_().pose.position, self.dig_location
        )

        # Create path from dig location to dump location.
        path_from_dig_to_dump = self.path_planner.find_path(
            self.dig_location, self.dump_location
        )
        # Create the path from dump location to dig location.
        path_from_dump_to_dig = path_from_dig_to_dump
        path_from_dump_to_dig.path = path_from_dump_to_dig.path[::-1]

        # Create path from the dump location to the charging station.
        path_from_dump_to_charge = self.path_planner.find_path(
            self.dump_location, self.lander_loc
        )

        # Calculate battery requirements for each path and add padding to each cost.
        # Add padding to pathing battery cost.
        # Padding is added to account for turning battery cost.
        # Padding is added to account for obstacle detection manuvers.
        battery_from_rover_to_dig = (
            (len(path_from_rover_to_dig.path) - 1)
            * self.move_battery_cost
            * 1.1
        )
        battery_from_dig_to_dump = (
            (len(path_from_dig_to_dump.path) - 1)
            * self.move_battery_cost_with_dirt
            * 1.1
        )
        battery_from_dump_to_dig = (
            (len(path_from_dig_to_dump.path) - 1) * self.move_battery_cost * 1.1
        )
        battery_from_dump_to_charge = (
            (len(path_from_dump_to_charge.path) - 1)
            * self.move_battery_cost
            * 1.1
        )

        # Calculate the battery cost of one action.
        # An action is the following sequence: Dig, go to dump with load, dump, go to
        # dig
        # A partial dig is counted as a total dig.
        battery_cost_of_one_action = self.dig_battery_cost
        battery_cost_of_one_action += battery_from_dig_to_dump
        battery_cost_of_one_action += self.dump_battery_cost
        battery_cost_of_one_action += battery_from_dump_to_dig

        # Calculate how many actions can be completed with the current battery.
        self.get_rover_status_()
        current_battery = self.battery

        # Negate the battery cost of the last action's pathing from the dump site to the
        # dig site.
        current_battery += battery_from_dump_to_dig
        actions = (
            current_battery
            - battery_from_rover_to_dig
            - battery_from_dump_to_charge
        ) // battery_cost_of_one_action

        # If we can't complete a single action, go to charge.
        if actions <= 0:
            rospy.loginfo(
                "Rover {} cannot complete any actions. Going to charge.".format(
                    self.id_
                )
            )
            # Update dig/dump locations available to Swarm Control.
            self.locations_assigned_lock.acquire()
            self.dig_locations_assigned.remove(self.dig_location)
            self.dump_locations_assigned.remove(self.dump_location)
            self.locations_assigned_lock.release()

            # Remove rover assignments.
            self.dig_location = None
            self.dump_location = None

            self.go_to_charge()

            return

        # Go to the digsite to begin action sequence.
        rospy.loginfo(
            "Rover {} is going to start action for {} actions.".format(
                self.id_,
                actions
                if actions
                < self.sub_pair_actions[(self.dig_location, self.dump_location)]
                else self.sub_pair_actions[
                    (self.dig_location, self.dump_location)
                ],
            )
        )

        # Get battery before pathing to recalculate actions possible if necessary.
        battery_before = self.battery
        self.go_to(self.dig_location, "digsite")

        # If we used more battery then expected recalculate actions.
        self.get_rover_status_()
        if self.battery + battery_from_rover_to_dig < battery_before:
            actions = (
                self.battery - battery_from_dump_to_charge
            ) // battery_cost_of_one_action

        # Level the assigned areas.
        while (
            actions > 0
            and self.sub_pair_actions[(self.dig_location, self.dump_location)]
        ):
            rospy.loginfo(
                "Rover {} is beginning action {}.".format(self.id_, actions)
            )

            # Dig at dig location.
            # TODO: Manage rover collisions at dig and dump locations.

            self.dig()

            # Get battery before pathing to recalculate pathing cost.
            self.get_rover_status_()
            battery_before = self.battery

            # Go to the dump site
            self.go_to(self.dump_location, "dumpsite")

            # Recalculate total actions possible and recalculate pathing cost to dump
            # site.
            (
                actions,
                battery_from_dig_to_dump,
                battery_cost_of_one_action,
            ) = self.battery_check(
                actions,
                battery_cost_of_one_action,
                battery_before,
                battery_from_dig_to_dump,
                battery_from_dump_to_charge,
            )
            self.dump()

            # Action is completed. Update sub pair actions.
            self.sub_pair_actions_lock.acquire()
            self.sub_pair_actions[(self.dig_location, self.dump_location)] -= 1
            self.sub_pair_actions_lock.release()
            # Update actions possible tracker.
            actions -= 1

            # If we have more actions to complete, go back to digsite.
            if (
                actions > 0
                and self.sub_pair_actions[
                    (self.dig_location, self.dump_location)
                ]
            ):

                # Get battery before pathing to recalculate pathing cost.
                self.get_rover_status_()
                battery_before = self.battery
                self.go_to(self.dig_location, "digsite")
                # Recalculate total actions possible and recalculate pathing cost to
                # dump site.
                (
                    actions,
                    battery_from_dump_to_dig,
                    battery_cost_of_one_action,
                ) = self.battery_check(
                    actions,
                    battery_cost_of_one_action,
                    battery_before,
                    battery_from_dump_to_dig,
                    battery_from_dump_to_charge,
                )

        rospy.loginfo(
            "Rover {} has completed possible actions.".format(self.id_)
        )
        # Allow Swarm Controller to assign these locations.
        self.locations_assigned_lock.acquire()
        self.dig_locations_assigned.remove(self.dig_location)
        self.dump_locations_assigned.remove(self.dump_location)
        self.locations_assigned_lock.release()

        # If the rover was not capable of completing all of its assigned actions, go
        # charge.
        if self.sub_pair_actions[(self.dig_location, self.dump_location)]:
            rospy.loginfo(
                "Rover {} could not complete assigned actions. Going to charge.".format(
                    self.id_
                )
            )
            self.go_to_charge()
        # If no actions are required of assigned subpair, delete subpair from master
        # data structure.
        else:
            self.sub_pair_actions_lock.acquire()
            del self.sub_pair_actions[(self.dig_location, self.dump_location)]
            self.sub_pair_actions_lock.release()

        # Set rover status to idle.
        self.activity = "idle"
        self.dig_location = None
        self.dump_location = None

    def run_rover(self):
        """Function run by the rover's thread.
        It runs the leveling routine after locations are assigned by SwarmController."""
        # Continue until signal to kill.
        while not self.kill_thread:
            # TODO: Too many ticks
            # Check for instruction to complete.
            # Level at the assigned locations.
            if self.activity == "ready to level":
                # Run leveling routine to level the assigned areas.
                rospy.loginfo("Rover {} beginning leveling".format(self.id_))
                try:
                    self.level()
                except RoverProblemError:
                    pass
                self.abort()
            # Execute a function provided by SwarmController.
            elif self.activity == "executing custom routine":
                try:
                    self.custom_routine(self)
                except RoverProblemError:
                    pass
                self.custom_routine = None
                self.abort()

        # Send rover home.
        try:
            self.go_to(self.home, "Spawn Point")
        except RoverProblemError:
            pass


class SwarmController:
    """Central controller for a swarm of EZRASSORs.

    Responsible for the scheduling and high-level path planning for each rover.
    """

    def __init__(
        self, robot_count, dig_sites, lander_loc, world, elevation_map
    ):
        # The number of robots to control.
        self.robot_count = robot_count
        # The queue which holds the ids of rovers who need instructions.
        self.rover_queue = Queue.Queue(robot_count)
        # Pre-assigned digsites.
        self.dig_sites = dig_sites
        # File path to world file.
        self.world = world
        # File path to elevation map.
        self.elevation_map = elevation_map
        # Coordinates of the lander.
        self.lander_loc = lander_loc

        # Instantiate rover objects
        self.rovers = {
            i: Rover(i, self.rover_queue, self.lander_loc)
            for i in range(1, robot_count + 1)
        }

    # Communicate to supervisor that a rover has been immobilized.
    def report_immobilized_rover(self, rover, immobilized_rover_list):
        # Send message
        rospy.loginfo(
            "Rover {} cannot move and requires attention at location {}, {}.".format(
                rover.id_, round(rover.position.x), round(rover.position.y)
            )
        )

        # Add id and location of immobilized rover to list.
        immobilized_rover_list.add(
            (
                rover.id_,
                (round(rover.position.x), round(rover.position.y)),
            )
        )

    def run(self):
        """Run Swarm Controller to command rovers to level an area on the moon
        given a set of instructions."""
        rospy.loginfo(
            "Running the swarm controller for {} rover(s)".format(
                self.robot_count
            )
        )

        # wait for rovers to spawn
        rospy.sleep(20.0)

        # Build path of the elevation map
        map_path = os.path.join(
            os.path.expanduser("~"),
            ".gazebo",
            "models",
            self.world,
            "materials",
            "textures",
            self.elevation_map,
        )

        # Read elevation map.
        height_matrix = imread(map_path, 0)  # Must be square and odd.
        height_matrix = np.array(height_matrix, dtype=int)

        # Crop portion of the map which will be leveled.
        # Create a matrix for the level area elevation map.
        rows, columns = height_matrix.shape
        origin = rows // 2  # The middle of the height matrix
        new_matrix_range = 21  # Length, width of cropped height matrix
        # Calculate the indexes of the new matrix in the original matrix.
        old_matrix_range = (
            origin - new_matrix_range // 2,
            origin + new_matrix_range // 2 + 1,
        )
        # Copy values from original matrix to new matrix.
        new_matrix = np.zeros((new_matrix_range, new_matrix_range), dtype=int)
        for row_num_height_matrix, row_num_new_height_matrix in zip(
            range(old_matrix_range[0], old_matrix_range[1]),
            range(new_matrix_range + 1),
        ):
            new_matrix[row_num_new_height_matrix] = height_matrix[
                row_num_height_matrix
            ][old_matrix_range[0] : old_matrix_range[1]]
        rospy.loginfo("Starting leveling algorithm.")

        # Run leveling algorithm to create the instructions to level.
        # An instruction is a sub pair and the number of actions to be performed to the
        # two.
        # A sub pair is a dig location and dump location where regolith will be
        # redistributed.
        dig_dump_pairs = create_level_instructions(pixel_normalize(new_matrix))

        # Data structure which holds locations which should not be assigned to rovers.
        blacklist = set()

        # Data structure which holds locations of immobilized rovers.
        immobilized_rover_list = set()

        # Data structures which holds all locations.
        dig_locations = set()
        dump_locations = set()

        # Create the data structure which holds the number of actions required for each
        # sub pair.
        # The key is the dig location and the dump location.
        # The value is the number of buckets full of dirt need to be transported from
        # the dig
        # location to the dump location.
        sub_pair_actions = {}
        for dig_location in dig_dump_pairs:
            dig_locations.add(dig_location)

            for dump_location, actions in dig_dump_pairs[dig_location]:
                sub_pair_actions[(dig_location, dump_location)] = math.ceil(
                    actions
                )

                dump_locations.add(dump_location)

        # Data structure to see which dig locations are being used and cannot be
        # assigned.
        dig_locations_assigned = set()

        # Data structure to see which dump locations are being used and cannot be
        # assigned.
        dump_locations_assigned = set()

        # Create A* path planner.
        path_planner = PathPlanner(map_path, rover_max_climb_slope=2)

        rospy.loginfo("Starting rover threads.")

        # Create locks to make global data structures thread safe.
        # Global data structes include dig and dump location sets and sub pair actions.
        sub_pair_actions_lock = Lock()
        locations_assigned_lock = Lock()

        # Start rover threads.
        # Add references of shared objects to rover.
        for rover in self.rovers.values():
            rover.get_rover_status_()
            rover.home = rover.position

            rover.path_planner = path_planner

            rover.sub_pair_actions = sub_pair_actions
            rover.sub_pair_actions_lock = sub_pair_actions_lock

            rover.dig_locations_assigned = dig_locations_assigned
            rover.dump_locations_assigned = dump_locations_assigned
            rover.locations_assigned_lock = locations_assigned_lock
            rover.thread.start()

        def check_actions_remaining():
            """Check if all instructions have been completed."""
            sub_pair_actions_lock.acquire()
            for dig_location, dump_location in sub_pair_actions:
                # If an instruction contain blacklisted locations, consider it
                # completed.
                if (
                    sub_pair_actions[(dig_location, dump_location)]
                    and (dig_location.x, dig_location.y) not in blacklist
                    and (dump_location.y, dump_location.x) not in blacklist
                ):
                    sub_pair_actions_lock.release()
                    return True
            sub_pair_actions_lock.release()
            return False

        # While instructions are still remaining to be completed,
        # give idle rovers instructions to complete.
        while check_actions_remaining():
            # If all rovers are immobilized, exit routine.
            if len(immobilized_rover_list) == len(self.rovers):
                break

            # Get an idle rover's id from the queue.
            rover = self.rovers[self.rover_queue.get()]

            # Check if rover encountered a problem.
            if rover.need_help != RoverState.okay.value:
                rospy.loginfo("CTM checking stuck Rover {}.".format(rover.id_))
                # Check what kind of problem the rover has.
                # The rover cannot move.
                if rover.need_help == RoverState.cannot_move.value:
                    rover.get_rover_status_()
                    rospy.loginfo(
                        "Rover {} cannot move. Adding ({}, {}) to blacklist.".format(
                            rover.id_,
                            round(rover.position.x),
                            round(rover.position.y),
                        )
                    )
                    # Blacklist location where rover is stuck.
                    blacklist.add(
                        (round(rover.position.x), round(rover.position.y))
                    )

                    self.report_immobilized_rover(rover, immobilized_rover_list)

                    # Kill rover thread to save processing power.
                    rover.kill_thread = True
                    rospy.loginfo("Killing Rover {}'s thread".format(rover.id_))

                    # Allow Swarm Controller to assign these locations.
                    rover.locations_assigned_lock.acquire()
                    rover.dig_locations_assigned.remove(rover.dig_location)
                    rover.dump_locations_assigned.remove(rover.dump_location)
                    rover.locations_assigned_lock.release()
                    rover.activity = "idle"
                    rover.dig_location = None
                    rover.dump_location = None
                    continue

                # Rover cannot reach target location.
                elif rover.need_help == RoverState.infinite_pathing.value:
                    # Get target location from rover activity.
                    rover_activity_list = rover.activity.split(" ")
                    # Add target location to blacklist.
                    blacklist.add(
                        (
                            int(rover_activity_list[-2]),
                            int(rover_activity_list[-1]),
                        )
                    )

                    rospy.loginfo(
                        "Rover {} inf pathing. Adding ({}, {}) to blacklist.".format(
                            rover.id_,
                            int(rover_activity_list[-2]),
                            int(rover_activity_list[-1]),
                        )
                    )
                    # Reset rover status.
                    rover.need_help = RoverState.okay.value

                    # If rover has a full drums, go dump its contents where it was
                    # before.
                    if rover.drum_full:

                        # Define a custom function to make rover reset its drums.
                        def custom_routine(rover):
                            rospy.loginfo(
                                "Starting custom routine for Rover {}".format(
                                    rover.id_
                                )
                            )
                            rover.go_to(rover.dig_location, "dig location")
                            rover.dump()
                            # Allow Swarm Controller to assign these locations.
                            rover.locations_assigned_lock.acquire()
                            rover.dig_locations_assigned.remove(
                                rover.dig_location
                            )
                            rover.dump_locations_assigned.remove(
                                rover.dump_location
                            )
                            rover.locations_assigned_lock.release()
                            # Set rover status to idle.
                            rover.activity = "idle"
                            rover.dig_location = None
                            rover.dump_location = None

                            rospy.loginfo(
                                "Finished custom_routine for Rover {}.".format(
                                    rover.id_
                                )
                            )

                        # Give rover instructions to complete.
                        rover.custom_routine = custom_routine
                        rover.activity = "executing custom routine"
                        rospy.loginfo(
                            "Finished checking on custom routine rover {}".format(
                                rover.id_
                            )
                        )
                        continue

                    # Allow Swarm Controller to assign these locations.
                    rover.locations_assigned_lock.acquire()
                    rover.dig_locations_assigned.remove(rover.dig_location)
                    rover.dump_locations_assigned.remove(rover.dump_location)
                    rover.locations_assigned_lock.release()
                    rover.activity = "idle"
                    rover.dig_location = None
                    rover.dump_location = None

            rospy.loginfo("Looking at tasks for Rover {}".format(rover.id_))

            # Assign rover dig and dump location.
            # Find the closest dig location to the rover and choose
            # a subpair with that location.
            rover.get_rover_status_()

            # Check if any dig locations and dump locations are available.
            # TODO Lock necessary?
            locations_assigned_lock.acquire()
            if len(dig_locations_assigned) == len(dig_locations) or len(
                dump_locations_assigned
            ) == len(dump_locations):
                locations_assigned_lock.release()
                # Requeue rover to assign it locations when they are available.
                # TODO: should we wait until there are locations not assigned before
                # continueing?
                self.rover_queue.put(rover.id_)
                continue
            locations_assigned_lock.release()

            # Find the closest dig location to the rover.
            closest_dig_distance = None
            assigned_dig_location = None
            assigned_dump_location = None
            sub_pair_actions_lock.acquire()

            for dig_location, dump_location in sub_pair_actions:
                # Make sure locations are not blacklisted.
                if blacklist:
                    if (dig_location.x, dig_location.y) in blacklist or (
                        dump_location.x,
                        dump_location.y,
                    ) in blacklist:
                        continue

                # Make sure locations are not assigned already.
                locations_assigned_lock.acquire()
                if (
                    dig_location in dig_locations_assigned
                    or dump_location in dump_locations_assigned
                ):
                    locations_assigned_lock.release()
                    continue
                locations_assigned_lock.release()

                # Ensure that chosen locations are not near other assigned locations.
                neighbors = [
                    (0, 1),
                    (0, -1),
                    (1, 0),
                    (-1, 0),
                    (1, 1),
                    (1, -1),
                    (-1, 1),
                    (-1, -1),
                ]
                for neighbor in neighbors:
                    dig_neighbor = Point()
                    dig_neighbor.x = dig_location.x + neighbor[0]
                    dig_neighbor.y = dig_location.y + neighbor[1]

                    dump_neighbor = Point()
                    dump_neighbor.x = dig_location.x + neighbor[0]
                    dump_neighbor.y = dig_location.y + neighbor[1]

                    locations_assigned_lock.acquire()
                    if (
                        dig_neighbor in dig_locations
                        or dump_neighbor in dump_locations
                    ):
                        locations_assigned_lock.release()
                        continue
                    locations_assigned_lock.release()

                # Choose sub pair if dig location is the closest dig location found so
                # far.
                distance = euclidean_distance(
                    dig_location.x,
                    rover.position.x,
                    dig_location.y,
                    rover.position.y,
                )
                if not closest_dig_distance or closest_dig_distance > distance:
                    closest_dig_distance = distance
                    assigned_dig_location, assigned_dump_location = (
                        dig_location,
                        dump_location,
                    )
            sub_pair_actions_lock.release()

            # Check if locations where successfully assigned.
            if not assigned_dig_location or not assigned_dump_location:
                self.rover_queue.put(rover.id_)
                continue

            rospy.loginfo("Assigned locations to Rover {}.".format(rover.id_))
            # Assign locations to rover.
            rover.dig_location = assigned_dig_location
            rover.dump_location = assigned_dump_location

            # Add locations to data structure so that they cannot be reasigned.
            locations_assigned_lock.acquire()
            dig_locations_assigned.add(rover.dig_location)
            dump_locations_assigned.add(rover.dump_location)
            locations_assigned_lock.release()
            # set the rover status to ready to dig.
            rover.activity = "ready to level"

        # After every instruction has been completed,
        # send the rovers home and kill thier thread.
        for rover in self.rovers.values():
            # TODO: Send rover's to home base. Do this in rover thread.
            rover.kill_thread = True
            rospy.loginfo("Killing Rover {}'s thread".format(rover.id_))

        if blacklist:
            rospy.loginfo("Blacklisted sites:")
            for element in blacklist:
                rospy.loginfo("{}".format(element))

        if immobilized_rover_list:
            rospy.loginfo("Immobilized Rover information:")
            for rover_id, location in immobilized_rover_list:
                rospy.loginfo(
                    "Rover {} is immobilized at {}".format(rover_id, location)
                )

        if len(immobilized_rover_list) != len(self.rovers) and not blacklist:
            rospy.loginfo("Area successfuly leveled!")


def on_start_up(
    robot_count, target_xs, target_ys, lander_coords, world, elevation_map
):
    """Initialization Function"""

    # ROS Node Init Parameters
    rospy.init_node("swarm_control")

    # Unpack digsite coordinates from string format
    target_xs = str(target_xs).split(" ")
    target_ys = str(target_ys).split(" ")

    lander_location = str(lander_coords).split(" ")

    lander_positionX = int(lander_location[0])
    lander_positionY = int(lander_location[1])

    lander_point = Point()
    lander_point.x = lander_positionX
    lander_point.y = lander_positionY

    if len(target_xs) != len(target_ys):
        raise ValueError(
            "Number of X coords doesn't match the number of Y coords"
        )

    # Create array of Point messages to store dig site locations
    dig_sites = []

    for i in range(len(target_xs)):
        coord = Point()
        coord.x = int(target_xs[i])
        coord.y = int(target_ys[i])

        dig_sites.append(coord)

    # Run the swarm control node
    swarm_controller = SwarmController(
        robot_count, dig_sites, lander_point, world, elevation_map
    )
    swarm_controller.run()
