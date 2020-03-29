#! /usr/bin/env python

import rospy
import actionlib
from ezrassor_swarm_control.msg import *
from geometry_msgs.msg import Point
from ezrassor_swarm_control.msg import Path


class WaypointClient:
    def __init__(self, robot_num):
        self.client_name = 'waypoint'
        self.namespace = rospy.get_namespace()

        # Create the waypoint action client
        self.client = actionlib.SimpleActionClient(self.client_name, waypointAction)

        self.client.wait_for_server()

        rospy.Subscriber('waypoint_client',
                         Path,
                         self.send_path)

    def send_waypoint(self, waypoint):

        # Create the goal to send to server
        goal = waypointGoal(target=waypoint)

        rospy.loginfo('digsite: {}'.format(waypoint))

        # Send the goal to the rover's waypoint server
        self.client.send_goal(goal)

        # Wait for the server to finish performing the action.
        self.client.wait_for_result()

        # Get the rover's final pose
        result = self.client.get_result()
        #rospy.loginfo('Current pose: {}'.format(result.final_pose))

    def send_path(self, data):
        path = data.path
        rospy.loginfo('Action client {} received path from {} to {}'.
                      format(self.namespace + self.client_name, (path[0].x, path[0].y), (path[-1].x, path[-1].y)))

        for node in path:
            self.send_waypoint(node)

        rospy.loginfo('Action client {} succeeded, path completed!'.format(self.namespace + self.client_name))

def on_start_up(robot_num):
    rospy.init_node('waypoint_client', anonymous=True)
    WaypointClient(robot_num)
    rospy.spin()

