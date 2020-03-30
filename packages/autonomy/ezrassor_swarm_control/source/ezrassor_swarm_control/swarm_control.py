import rospy
from std_msgs.msg import Int8
from geometry_msgs.msg import Point

from ezrassor_swarm_control.msg import Path

from path_planner import PathPlanner

from swarm_utils import get_rover_status, preempt_rover_path, euclidean_distance


import os

class Rover :
    def __init__(self, id) :
        self.id = id
        self.path_in_progress = False
        self.path_completed = False
        self.battery = 100


class SwarmController:
    def __init__(self, robot_count, dig_sites, lander_loc):
        self.robot_count = robot_count
        self.dig_sites = dig_sites
        self.lander_loc = lander_loc
        self.rover_activity_status_db = [Rover(i) for i in range(10)]

        self.waypoint_pubs = {i: rospy.Publisher('/ezrassor{}/waypoint_client'.format(i),
                                                 Path,
                                                 queue_size=10)
                              for i in range(1, robot_count+1)}

    def is_at_lander(self, robot_status) :
        if euclidean_distance(robot_status.pose.position.x, self.lander_loc.x, robot_status.pose.position.y, self.lander_loc.y) <= 1 :
            return True
        else :
            return False
    
    def is_at_digsite(self, robot_status) :
        if euclidean_distance(robot_status.pose.position.x, self.dig_sites[0].x, robot_status.pose.position.y, self.dig_sites[0].y) <= 1 :
            return True
        else :
            return False
   
    def run(self):
        rospy.loginfo('Running the swarm controller for {} rover(s)'.format(self.robot_count))
        rospy.loginfo('{} total dig sites: {}'
                      .format(len(self.dig_sites), [(site.x, site.y) for site in self.dig_sites]))
        
        # wait for rovers to spawn
        rospy.sleep(5.)

        height_map = os.path.join(os.path.expanduser('~'),
                                  '.gazebo', 'models', 'random', 'materials', 'textures', 'random_map.jpg')

        path_planner = PathPlanner(height_map, rover_max_climb_slope=1)

        
        while True :
            for i in range(1, self.robot_count + 1) :
                rover_status = get_rover_status(i)
                rospy.loginfo(self.rover_activity_status_db[i].path_in_progress)

                if rover_status :
                    if not self.is_at_digsite(rover_status) :
                        if not self.rover_activity_status_db[i].path_in_progress :
                            path = path_planner.find_path(rover_status.pose.position, self.dig_sites[0])
                            if path :
                                self.waypoint_pubs[i].publish(path)
                                self.rover_activity_status_db[i].path_in_progress = True
        
                    else :
                        if self.rover_activity_status_db[i].path_in_progress :
                            self.rover_activity_status_db[i].path_in_progress = False

                        rospy.loginfo("FROM SWARM CONTROLLER: rover at dig site!")
             rospy.sleep(5.)
             preempt_rover_path(1)
                   

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
