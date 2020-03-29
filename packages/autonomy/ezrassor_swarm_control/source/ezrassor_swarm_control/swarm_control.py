import rospy
from std_msgs.msg import Int8
from geometry_msgs.msg import Point

from ezrassor_swarm_control.msg import Path

from path_planner import PathPlanner
from swarm_utils import get_rover_status
from swarm_utils import euclidean_distance

import os


class SwarmController:
    def __init__(self, robot_count, dig_sites, lander_loc):
        self.robot_count = robot_count
        self.dig_sites = dig_sites
        self.lander_loc = lander_loc

        self.waypoint_pubs = {i: rospy.Publisher('/ezrassor{}/waypoint_client'.format(i),
                                                 Path,
                                                 queue_size=10)
                              for i in range(1, robot_count+1)}

    def run(self):
        rospy.loginfo('Running the swarm controller for {} rover(s)'.format(self.robot_count))
        rospy.loginfo('{} total dig sites: {}'
                      .format(len(self.dig_sites), [(site.x, site.y) for site in self.dig_sites]))
        
        rospy.loginfo('FROM SWARM CONTROLLER: lander location: {}'.format(self.lander_loc))

        # wait for rovers to spawn
        rospy.sleep(5.)

        height_map = os.path.join(os.path.expanduser('~'),
                                  '.gazebo', 'models', 'random', 'materials', 'textures', 'random_map.jpg')

        path_planner = PathPlanner(height_map, rover_max_climb_slope=1)


        while(True) :
            for i in range(1, self.robot_count + 1) :
                status = get_rover_status(i)
                # rospy.loginfo(status)
                rospy.sleep(1.)
                if status is not None :
                    dist_from_digsite = euclidean_distance(status.pose.position.x, self.dig_sites[0].x, status.pose.position.y, self.dig_sites[0].y)
                    rospy.loginfo('FROM SWARM CONTROLLER: distance from dig site: {}'.format(dist_from_digsite))
                    if dist_from_digsite > 0.5 :
                        path = path_planner.find_path(status.pose.position, self.dig_sites[0])
                        if path is not None :
                            self.waypoint_pubs[i].publish(path)
                    else :
                        pass

        # for i in range(1, self.robot_count + 1):
            # Get status (battery and pose) of rover using status service
            # status = get_rover_status(i)
            # rospy.loginfo(status)

            # if status is not None:
            #     # Find path
            #     path = path_planner.find_path(status.pose.position, self.dig_sites[0])

            #     # Send rover along path
            #     if path is not None:
            #         self.waypoint_pubs[i].publish(path)


def on_start_up(robot_count, target_xs, target_ys, lander_coords):
    """ Initialization Function  """

    # ROS Node Init Parameters
    rospy.init_node('swarm_control')

    # Unpack digsite coordinates from string format
    target_xs = str(target_xs).split(' ')
    target_ys = str(target_ys).split(' ')

    lander_location = str(lander_coords).split(' ')

    lander_positionX = int(lander_location[0])
    lander_positionY = int(lander_location[1])

    lander_point = Point()
    lander_point.x = lander_positionX
    lander_point.y = lander_positionY


    if len(target_xs) != len(target_ys):
        raise ValueError('Number of dig site x coordinates does not match the number of y coordinates')

    # Creat array of Point messages to store dig site locations
    dig_sites = []

    for i in range(len(target_xs)):
        coord = Point()
        coord.x = int(target_xs[i])
        coord.y = int(target_ys[i])

        dig_sites.append(coord)

    swarm_controller = SwarmController(robot_count, dig_sites, lander_point)
    swarm_controller.run()
