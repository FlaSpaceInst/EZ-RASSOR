#! /usr/bin/env python

import os

import rospy
import actionlib

from path_planner import PathPlanner
from swarm_utils import euclidean2D
from ezrassor_swarm_control.msg import waypointAction, waypointGoal
from ezrassor_swarm_control.msg import Path

from ezrassor_swarm_control.srv import PreemptPath, PreemptPathResponse

from constants import commands

from std_msgs.msg import Int8
from enum import Enum

# Enumerate status codes which rover objects will use to differentiate which actions have been completed.
class ActionCodes(Enum):
    path_completed = 1
    action_completed = 2
    preempted = 3


class WaypointClient:
    """
    Communication layer between the central swarm controller and each EZ-RASSOR
    Implemented as a ROS action client-server API
    """

    def __init__(self, robot_num, world, elevation_map):
        self.client_name = "waypoint"
        self.namespace = rospy.get_namespace()

        # Create the waypoint action client
        self.client = actionlib.SimpleActionClient(
            self.client_name, waypointAction
        )

        self.client.wait_for_server()

        rospy.Subscriber("waypoint_client", Path, self.send_path)

        # Publisher to notify rover that the action was completed.
        self.action_completed_pub = rospy.Publisher(
            "waypoint_client_action_completed", Int8, queue_size=10
        )

        # Service which allows the waypoint client's current path to be
        # preempted from any other node
        self.preempt_service = rospy.Service(
            "preempt_path", PreemptPath, self.preempt_path
        )

        # Whether a path has been canceled or not
        self.preempt = False

        # Waypoint the rover is currently moving towards
        self.cur_waypoint = None

        # Ultimate goal the rover is moving towards (usually a dig site or
        # charging station)
        self.goal = None

        # Max distance a rover can veer from its current waypoint before a new
        # path is generated
        self.max_veer_distance = 2

        # Create path planner to be used during replanning
        height_map = os.path.join(
            os.path.expanduser("~"),
            ".gazebo",
            "models",
            world,
            "materials",
            "textures",
            elevation_map,
        )

        self.path_planner = PathPlanner(height_map, rover_max_climb_slope=2)

    def is_dig_cmd(self, cmd):
        return cmd == commands["DIG"]

    def is_charge_cmd(self, cmd):
        return cmd == commands["CHG"]

    def is_dump_cmd(self, cmd):
        return cmd == commands["DUMP"]

    def preempt_path(self, request=None):
        """Handle preempt path requests."""

        self.preempt = True
        self.client.cancel_goal()

        return PreemptPathResponse()

    def feedback_cb(self, feedback):
        """Handle rover feedback."""

        # Replan path if rover veers away from its current waypoint too far
        if (
            not (
                self.is_dig_cmd(self.goal)
                or self.is_charge_cmd(self.goal)
                or self.is_dump_cmd(self.goal)
            )
            and self.goal
            and self.cur_waypoint
        ):
            veer_distance = euclidean2D(
                feedback.pose.position, self.cur_waypoint
            )

            if veer_distance > self.max_veer_distance:
                rospy.loginfo(
                    "Client {} forced to replan path!".format(
                        self.namespace + self.client_name
                    )
                )
                self.preempt_path()

                new_path = self.path_planner.find_path(
                    feedback.pose.position, self.goal
                )

                # Wait for previous path to be totally cancelled before sending
                # the new path
                while self.preempt is True:
                    rospy.sleep(1.0)

                self.send_path(new_path)

    def done_cb(self, status, result):
        """Handle interruption or completion of request."""

        x = round(result.pose.position.x, 2)
        y = round(result.pose.position.y, 2)

        if result.preempted:
            rospy.loginfo(
                "Client {} preempted! (position: {} battery: {})".format(
                    (self.namespace + self.client_name), (x, y), result.battery
                )
            )
            return

        rospy.loginfo(
            "Client {} reached waypoint! (position: {} battery: {})".format(
                (self.namespace + self.client_name), (x, y), result.battery
            )
        )

    def send_waypoint(self, waypoint):

        # Create the goal to send to server
        goal = waypointGoal(target=waypoint)

        # Send the goal to the rover's waypoint server
        self.client.send_goal(
            goal, done_cb=self.done_cb, feedback_cb=self.feedback_cb
        )

        # Wait for the server to finish performing the action.
        self.client.wait_for_result()

    def send_path(self, data):
        """Callback executed when the waypoint client receives a path"""

        path = data.path

        # Set rover's ultimate goal as the path's last waypoint
        self.goal = path[-1]

        if self.is_dig_cmd(self.goal):
            rospy.loginfo(
                "Client {} received dig command!".format(
                    self.namespace + self.client_name
                )
            )
            self.send_waypoint(self.goal)

            # Publish that an action has been completed by the waypoint client.
            self.action_completed_pub.publish(
                ActionCodes.action_completed.value
            )

        elif self.is_charge_cmd(self.goal):
            rospy.loginfo(
                "Client {} received charge command!".format(
                    self.namespace + self.client_name
                )
            )
            self.send_waypoint(self.goal)
            # Publish that an action has been completed by the waypoint client.
            self.action_completed_pub.publish(
                ActionCodes.action_completed.value
            )

        elif self.is_dump_cmd(self.goal):
            rospy.loginfo(
                "Client {} received dump command!".format(
                    self.namespace + self.client_name
                )
            )
            self.send_waypoint(self.goal)

            # Publish that an action has been completed by the waypoint client.
            self.action_completed_pub.publish(
                ActionCodes.action_completed.value
            )

        else:
            rospy.loginfo(
                "Client {} received path from {} to {}".format(
                    self.namespace + self.client_name,
                    (path[0].x, path[0].y),
                    (path[-1].x, path[-1].y),
                )
            )

            # Send each waypoint in a path to the rover
            for node in path[1:]:
                # If request was canceled stop sending waypoints
                if self.preempt:
                    break

                # Sends the waypoint to a rover
                self.cur_waypoint = node
                self.send_waypoint(node)

            if not self.preempt:
                # Publish that the rover has arrived at its destination.
                self.action_completed_pub.publish(
                    ActionCodes.path_completed.value
                )
            else:
                # Publish that the rover path was preempted.
                self.action_completed_pub.publish(ActionCodes.preempted.value)

        # probably can remove this.
        rospy.loginfo(
            "Client {} finished sending waypoints!".format(
                self.namespace + self.client_name
            )
        )

        # Reset server to receive another path
        self.preempt = False
        self.cur_waypoint = None
        self.goal = None


def on_start_up(robot_num, world, elevation_map):
    rospy.init_node("waypoint_client")
    WaypointClient(robot_num, world, elevation_map)
    rospy.spin()
