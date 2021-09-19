import rospy
from geometry_msgs.msg import Point

from constants import commands

from ezrassor_swarm_control.msg import Path

from path_planner import PathPlanner
from swarm_utils import euclidean_distance
from swarm_utils import get_rover_status, preempt_rover_path

import os


class Rover:
    """Abstract class for the controller's knowledge of an EZRASSOR."""

    #@staticmethod
    def create_command(self, cmd):
        path = Path()
        path.path.append(commands[cmd])
        return path

    def __init__(self, id_):
        self.id_ = id_
        self.activity = "idle"
        self.path_completed = False
        self.battery = 100
        self.position = None

        self.waypoint = rospy.Publisher(
                "/ezrassor{}/waypoint_client".format(self.id_), Path, queue_size=10
            )

    def get_rover_status_(self):
        rover_status = get_rover_status(self.id_)
        self.position = rover_status.pose.position
        self.battery = rover_status.battery
        return rover_status

    def preempt_rover_path(self):
        preempt_rover_path(self.id_)

    def dig(self):
        self.waypoint.publish(
            self.create_command("DIG")
        )
        self.activity = "digging"

    def dump(self):
        self.waypoint.publish(
            self.create_command("DUMP")
        )
        self.activity = "dumping"

    def charge(self):
        self.waypoint.publish(
            self.create_command("CHG")
        )
        self.activity = "charging"

    def go_to(self, target_point, target_name, path_planner):
        path = path_planner.find_path(
            self.get_rover_status_().pose.position, target_point
        )
        if path:

            self.waypoint.publish(path)
            self.activity = "driving to {}".format(target_name)

class SwarmController:
    """Central controller for a swarm of EZRASSORs.

    Responsible for the scheduling and high-level path planning for each rover.
    """

    def __init__(
        self, robot_count, dig_sites, lander_loc, world, elevation_map
    ):
        self.robot_count = robot_count
        self.dig_sites = dig_sites
        self.world = world
        self.elevation_map = elevation_map
        self.lander_loc = lander_loc

        # Instaniate rover objects
        self.rovers = {
            i: Rover(i) for i in range(1, robot_count + 1)
        }

        """
        # Tracks the battery and activity of rovers
        self.rover_activity_status_db = {
            i: Rover(i) for i in range(1, robot_count + 1)
        }
        """
        """
        # Set up publishers used to send paths to each rover's waypoint client
        self.waypoint_pubs = {
            i: rospy.Publisher(
                "/ezrassor{}/waypoint_client".format(i), Path, queue_size=10
            )
            for i in range(1, robot_count + 1)
        }
        """

    def is_at_lander(self, robot_status):
        if (
            euclidean_distance(
                robot_status.pose.position.x,
                self.lander_loc.x,
                robot_status.pose.position.y,
                self.lander_loc.y,
            )
            <= 0.5
        ):
            return True
        else:
            return False

    def is_at_digsite(self, robot_status, dig_site_idx):
        if (
            euclidean_distance(
                robot_status.pose.position.x,
                self.dig_sites[dig_site_idx].x,
                robot_status.pose.position.y,
                self.dig_sites[dig_site_idx].y,
            )
            <= 0.5
        ):
            return True
        else:
            return False

    """
    def create_command(self, cmd):
        path = Path()
        path.path.append(commands[cmd])
        return path
    """

    def run(self):
        rospy.loginfo(
            "Running the swarm controller for {} rover(s)".format(
                self.robot_count
            )
        )
        rospy.loginfo(
            "{} total dig sites: {}".format(
                len(self.dig_sites),
                [(site.x, site.y) for site in self.dig_sites],
            )
        )

        # wait for rovers to spawn
        rospy.sleep(20.0)

        # Build path of the elevation map
        height_map = os.path.join(
            os.path.expanduser("~"),
            ".gazebo",
            "models",
            self.world,
            "materials",
            "textures",
            self.elevation_map,
        )

        # Create A* path planner
        path_planner = PathPlanner(height_map, rover_max_climb_slope=2)

        while True:
            for rover in self.rovers.values():
                rover_status = rover.get_rover_status_()
                if rover_status:
                    site_num = (rover.id_ - 1) % len(self.dig_sites)

                    if rover.activity == "idle":
                        rover.go_to(self.dig_sites[site_num], "digsite", path_planner)

                    elif rover.activity == "digging":
                        if rover.battery <= 35.0:
                            rover.preempt_rover_path()
                            rover.go_to(self.lander_loc, "lander", path_planner)

                    elif rover.activity == "driving to lander":
                        if self.is_at_lander(rover_status):
                            if rover.activity != "charging":
                                rover.preempt_rover_path()
                                rover.charge()

                    elif rover.activity == "charging":
                        if rover.battery >= 95.0:
                            rover.go_to(self.dig_sites[site_num], "digsite", path_planner)
                    else:
                        if self.is_at_digsite(rover_status, site_num):
                            if rover.activity == "driving to digsite":
                                if rover.battery >= 35.0:
                                    rover.preempt_rover_path()
                                    rover.dig()

            """
            for i in range(1, self.robot_count + 1):
                rover_status = get_rover_status(i)

                if rover_status:
                    site_num = (i - 1) % len(self.dig_sites)
                    if self.rover_activity_status_db[i].activity == "idle":
                        path = path_planner.find_path(
                            rover_status.pose.position, self.dig_sites[site_num]
                        )
                        if path:
                            self.waypoint_pubs[i].publish(path)
                            self.rover_activity_status_db[
                                i
                            ].activity = "driving to digsite"
            """
            """
                    elif self.rover_activity_status_db[i].activity == "digging":
                        if rover_status.battery <= 35.0:
                            preempt_rover_path(i)
                            path = path_planner.find_path(
                                rover_status.pose.position, self.lander_loc
                            )
                            if path:
                                self.waypoint_pubs[i].publish(path)
                                self.rover_activity_status_db[
                                    i
                                ].activity = "driving to lander"
            """
            """
                    elif (
                        self.rover_activity_status_db[i].activity
                        == "driving to lander"
                    ):
                        if self.is_at_lander(rover_status):
                            if (
                                self.rover_activity_status_db[i].activity
                                != "charging"
                            ):
                                preempt_rover_path(i)
                                self.waypoint_pubs[i].publish(
                                    self.create_command("CHG")
                                )
                                self.rover_activity_status_db[
                                    i
                                ].activity = "charging"
            """
            """

                    elif (
                        self.rover_activity_status_db[i].activity == "charging"
                    ):
                        if rover_status.battery >= 95.0:
                            path = path_planner.find_path(
                                rover_status.pose.position,
                                self.dig_sites[site_num],
                            )
                            if path:
                                self.waypoint_pubs[i].publish(path)
                                self.rover_activity_status_db[
                                    i
                                ].activity = "driving to digsite"
            """
            """
                    else:
                        if self.is_at_digsite(rover_status, site_num):
                            if (
                                self.rover_activity_status_db[i].activity
                                == "driving to digsite"
                            ):
                                if rover_status.battery >= 35.0:
                                    preempt_rover_path(i)
                                    self.waypoint_pubs[i].publish(
                                        self.create_command("DIG")
                                    )
                                    self.rover_activity_status_db[
                                        i
                                    ].activity = "digging"
            """


def on_start_up(
    robot_count, target_xs, target_ys, lander_coords, world, elevation_map
):
    """ Initialization Function  """

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

    # Creat array of Point messages to store dig site locations
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
