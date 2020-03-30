#! /usr/bin/env python

import rospy
import actionlib
from ezrassor_swarm_control.msg import *
from geometry_msgs.msg import Point
from ezrassor_swarm_control.msg import Path

from ezrassor_swarm_control.srv import PreemptPath, PreemptPathResponse


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

        self.preempt_service = rospy.Service('preempt_path', PreemptPath, self.preempt_path)

        # Minimum amount of battery a rover needs to continue along a path
        self.min_battery_needed = 20

        # Whether a path has been canceled or not
        self.preempt = False

    def preempt_path(self, request):
        self.preempt = True
        self.client.cancel_goal()

        return PreemptPathResponse()

    def feedback_cb(self, feedback):
        """Callback executed when the waypoint client receives feedback from a rover"""

        x = round(feedback.pose.position.x, 2)
        y = round(feedback.pose.position.y, 2)

        #rospy.loginfo('Waypoint client {} received feedback! position: {} battery: {}'.
        #              format((self.namespace + self.client_name), (x, y), feedback.battery))

        # Tell rover to stop trying to reach its current waypoint if it's low on battery
        if feedback.battery < self.min_battery_needed:
            self.preempt = True
            self.client.cancel_goal()

            """PLAN NEW PATH TO A CHARGING STATION AND SEND IT TO THE ROVER"""

        # Replan if rover veers off its path too much
        """IMPLEMENT REPLANNING HERE"""

    def done_cb(self, status, result):
        """Callback executed when the rover reaches a waypoint or the request is preempted"""

        x = round(result.pose.position.x, 2)
        y = round(result.pose.position.y, 2)

        if result.preempted:
            rospy.loginfo('Waypoint client {} preempted! Current position: {} Current battery: {}'.
                          format((self.namespace + self.client_name), (x, y), result.battery))
        else:
            rospy.loginfo('Waypoint client {} reached waypoint! Current position: {} Current battery: {}'.
                          format((self.namespace + self.client_name), (x, y), result.battery))


    def send_waypoint(self, waypoint):

        # Create the goal to send to server
        goal = waypointGoal(target=waypoint)

        rospy.loginfo('digsite: {}'.format(waypoint))

        # Send the goal to the rover's waypoint server
        self.client.send_goal(goal, done_cb=self.done_cb, feedback_cb=self.feedback_cb)

        # Wait for the server to finish performing the action.
        self.client.wait_for_result()

    def send_path(self, data):
        """Callback executed when the waypoint client receives a path"""

        path = data.path
        rospy.loginfo('Waypoint client {} received path from {} to {}'.
                      format(self.namespace + self.client_name, (path[0].x, path[0].y), (path[-1].x, path[-1].y)))

        # Send each waypoint in a path to the rover
        for node in path:
            if self.preempt:
                # If request was canceled, reset server to receive a new request in the future
                self.preempt = False

                # Return to stop sending waypoints
                return

            # Sends the waypoint to a rover
            self.send_waypoint(node)

        rospy.loginfo('Waypoint client {} succeeded, path completed!'.format(self.namespace + self.client_name))


def on_start_up(robot_num):
    rospy.init_node('waypoint_client', anonymous=True)
    WaypointClient(robot_num)
    rospy.spin()
