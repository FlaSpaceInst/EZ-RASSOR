import rospy
from std_msgs.msg import Int8
from geometry_msgs.msg import Point

from ezrassor_swarm_control.msg import Path

from path_planner import PathPlanner
from swarm_utils import get_rover_status

import os


class SwarmController:
    def __init__(self, robot_count, dig_sites):
        self.robot_count = robot_count
        self.dig_sites = dig_sites

        self.waypoint_pubs = {i: rospy.Publisher('/ezrassor{}/waypoint_client'.format(i),
                                                 Path,
                                                 queue_size=10)
                              for i in range(1, robot_count+1)}

    def run(self):
        rospy.loginfo('Running the swarm controller for {} rover(s)'.format(self.robot_count))
        rospy.loginfo('{} total dig sites: {}'
                      .format(len(self.dig_sites), [(site.x, site.y) for site in self.dig_sites]))

        # wait for rovers to spawn
        rospy.sleep(5.)

        height_map = os.path.join(os.path.expanduser('~'),
                                  '.gazebo', 'models', 'random', 'materials', 'textures', 'random_map.jpg')

        path_planner = PathPlanner(height_map, rover_max_climb_slope=1)

        for i in range(1, self.robot_count + 1):
            # Get status (battery and pose) of rover using status service
            status = get_rover_status(i)

            if status is not None:
                # Find path
                path = path_planner.find_path(status.pose.position, self.dig_sites[0])

                # Send rover along path
                if path is not None:
                    self.waypoint_pubs[i].publish(path)


def on_start_up(robot_count, target_xs, target_ys):
    """ Initialization Function  """

    # ROS Node Init Parameters
    rospy.init_node('swarm_control')

    # Unpack digsite coordinates from string format
    target_xs = str(target_xs).split(' ')
    target_ys = str(target_ys).split(' ')

    if len(target_xs) != len(target_ys):
        raise ValueError('Number of dig site x coordinates does not match the number of y coordinates')

    # Creat array of Point messages to store dig site locations
    dig_sites = []

    for i in range(len(target_xs)):
        coord = Point()
        coord.x = int(target_xs[i])
        coord.y = int(target_ys[i])

        dig_sites.append(coord)

    swarm_controller = SwarmController(robot_count, dig_sites)
    swarm_controller.run()
