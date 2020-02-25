import rospy
import sys

from std_msgs.msg import Int8
from geometry_msgs.msg import Point


class SwarmController:
    def __init__(self, robot_count, dig_sites):
        self.robot_count = robot_count
        self.dig_sites = dig_sites

        self.rover_control_pubs = [rospy.Publisher('ezrassor{}/autonomous_toggles'.format(i),
                                                   Int8,
                                                   queue_size=10)
                                   for i in range(1, self.robot_count + 1)]

        self.rover_target_pubs = [rospy.Publisher('ezrassor{}/rover_target'.format(i),
                                                  Point,
                                                  queue_size=10)
                                  for i in range(1, self.robot_count + 1)]

    def run(self):
        rospy.loginfo('Running the swarm controller for {} rover(s)'.format(self.robot_count))
        rospy.loginfo('{} total dig sites: {}'
                      .format(len(self.dig_sites), [(site.x, site.y) for site in self.dig_sites]))

        # wait for rovers to spawn
        rospy.sleep(8.)

        # Simply testing out publishing rover target topic
        self.rover_target_pubs[0].publish(self.dig_sites[0])
        self.rover_control_pubs[0].publish(1)
        rospy.sleep(17.)
        # Dig
        self.rover_control_pubs[0].publish(2)
        rospy.sleep(12.)
        # Return to 0,0
        home = Point()
        home.x = 5.
        home.y = 0.
        self.rover_target_pubs[0].publish(home)
        self.rover_control_pubs[0].publish(1)


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
        coord.x = float(target_xs[i])
        coord.y = float(target_ys[i])

        dig_sites.append(coord)

    swarm_controller = SwarmController(robot_count, dig_sites)
    swarm_controller.run()
