import rospy
from geometry_msgs.msg import Point

from constants import commands

from ezrassor_swarm_control.msg import Path

from path_planner import PathPlanner
from swarm_utils import euclidean_distance
from swarm_utils import get_rover_status, preempt_rover_path, update_rover_status

import os


class Rover:
    """Abstract class for the controller's knowledge of an EZRASSOR."""

    def __init__(self, id):
        self.id = id
        self.activity = "idle"
        self.path_completed = False
        self.battery = 100


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

        # Set up publishers used to send paths to each rover's waypoint client
        self.waypoint_pubs = {
            i: rospy.Publisher(
                "/ezrassor{}/waypoint_client".format(i), Path, queue_size=10
            )
            for i in range(1, robot_count + 1)
        }

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

    def is_at_digsite(self, robot_status, site):
        if (
            euclidean_distance(
                robot_status.pose.position.x,
                site.x,
                robot_status.pose.position.y,
                site.y,
            )
            <= 0.5
        ):
            return True
        else:
            return False

    def create_command(self, cmd):
        path = Path()
        path.path.append(commands[cmd])
        return path

    def update_digsites(self, open_sites, ID, add = False):
        """ Updates list of available digsites """
        
        #If add is true, ID is a rover ID
        if add is True:
            open_sites.append(get_rover_status(ID).digsite)
            rospy.loginfo("Updated digsites")
            rospy.loginfo(open_sites)
            return open_sites

        if open_sites == []:
            open_sites = list(self.dig_sites)

        #Otherwise, ID is a siteID
        open_sites.pop(ID)
        rospy.loginfo("Available digsites")
        rospy.loginfo(open_sites)
        return open_sites

    def closest_digsite(self, open_dig_sites, rover_status):
        """ Determines the closest digsite to a rover """
        dist_min = -1
        curr_min = -1

        for i in range(0, len(open_dig_sites)):
            dist = euclidean_distance(
                rover_status.pose.position.x, open_dig_sites[i].x,
                rover_status.pose.position.y, open_dig_sites[i].y
            )
            if curr_min == -1:
                curr_min = i
                dist_min = dist
            elif dist < dist_min:
                dist_min = dist
                curr_min = i
        #returns the index of closest site to rover from the open_dig_sites list
        return curr_min

    def assignPath(self, roverID, target, rover_status, path_planner):
        """ Calculates a path from a Rover to a target location and assigns it """
        path = path_planner.find_path(rover_status.pose.position, target)
        if path:
            self.waypoint_pubs[roverID].publish(path)

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

        # Task Scheduling
        available_sites = list(self.dig_sites)
        while True:            
            for i in range(1, self.robot_count + 1):

                #retrieve rover information for decision making
                rover_status = get_rover_status(i)

                #battery check
                if rover_status.battery < 35.0 and rover_status.activity not in ["driving to lander", "charging"]:

                    rospy.loginfo("LOW BATTERY : Rover " + str(i))
                    preempt_rover_path(i)
                    self.assignPath(i, self.lander_loc, rover_status, path_planner)
                    self.update_digsites(available_sites, i, add = True)
                    update_rover_status(i, "driving to lander", Point())

                elif rover_status.battery > 95.0 and rover_status.activity == "charging":
                    update_rover_status(i, "idle", rover_status.digsite)

                #digsite assignment, only assigns if activity is "idle"
                if rover_status.activity == "idle":

                    #determine closest site to rover and send it
                    closest_siteID = self.closest_digsite(available_sites, rover_status)
                    self.assignPath(i, available_sites[closest_siteID], rover_status, path_planner)

                    #update status with activity and assigned site. Also update available sites.
                    update_rover_status(i, "driving to digsite", available_sites[closest_siteID])
                    available_sites = self.update_digsites(available_sites, closest_siteID)

                #determine if rover has reached destination and send an action command
                elif rover_status.activity == "driving to lander" and self.is_at_lander(rover_status):
                    
                    rospy.loginfo("CHARGING: Rover " + str(i))
                    preempt_rover_path(i)
                    self.waypoint_pubs[i].publish(self.create_command("CHG"))
                    update_rover_status(i, "charging", rover_status.digsite)

                elif rover_status.activity == "driving to digsite" and self.is_at_digsite(rover_status, rover_status.digsite):
                  
                    rospy.loginfo("DIGGING: Rover " + str(i))
                    preempt_rover_path(i)
                    self.waypoint_pubs[i].publish(self.create_command("DIG"))
                    update_rover_status(i, "digging", rover_status.digsite)


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
