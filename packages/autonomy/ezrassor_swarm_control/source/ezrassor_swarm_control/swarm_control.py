import rospy
import sys

from geometry_msgs.msg import Point

class SwarmController:
    def __init__(self, robot_count, dig_sites):
        self.robot_count = robot_count
        self.dig_sites = dig_sites

    def run(self):
        rospy.loginfo('Running the swarm controller for {} rover(s)'.format(self.robot_count))
        rospy.loginfo('{} total dig sites: {}'
                      .format(len(self.dig_sites), [(site.x, site.y) for site in self.dig_sites]))

def on_start_up(robot_count, target_xs, target_ys):
    """ Initialization Function  """

    # ROS Node Init Parameters
    rospy.init_node('swarm_control')

    # Unpack digsite coordinates from string format
    target_xs = target_xs.split(' ')
    target_ys = target_ys.split(' ')

    if len(target_xs) != len(target_ys):
        raise ValueError('Number of dig site x coordinates does not match the number of y coordinates')

    # Creat array of Point messages to store dig site locations
    dig_sites = []

    for i in range(len(target_xs)):
        coord = Point()
        coord.x = float(target_xs[i])
        coord.y = float(target_ys[i])

        dig_sites.append(coord)

    swarm_controller = SwarmController(robot_count, dig_sites)
    swarm_controller.run()
