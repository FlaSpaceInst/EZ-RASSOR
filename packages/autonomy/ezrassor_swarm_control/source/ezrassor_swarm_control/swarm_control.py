import rospy
from std_msgs.msg import Int8
from geometry_msgs.msg import Point
from ezrassor_swarm_control.msg import Path

from path_planner import PathPlanner


class SwarmController:
    def __init__(self, robot_count, dig_sites):
        self.robot_count = robot_count
        self.dig_sites = dig_sites

        self.waypoint_pubs = [rospy.Publisher('/ezrassor{}/waypoint_client'.format(i),
                                                 Path,
                                                 queue_size=10)
                              for i in range(1, robot_count+1)]

    def run(self):
        rospy.loginfo('Running the swarm controller for {} rover(s)'.format(self.robot_count))
        rospy.loginfo('{} total dig sites: {}'
                      .format(len(self.dig_sites), [(site.x, site.y) for site in self.dig_sites]))

        # wait for rovers to spawn
        rospy.sleep(4.)

        height_map = '/home/danielzgsilva/.gazebo/models/random/materials/textures/random_map.jpg'
        path_planner = PathPlanner(height_map, rover_max_climb_slope=1)

        start = Point()
        start.x = 0
        start.y = 0

        # Find path
        path = path_planner.find_path(start, self.dig_sites[0])

        # Send rover 1 along path
        if path is not None:
            self.waypoint_pubs[0].publish(path)

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
