#!/usr/bin/env python

import rospy
import actionlib
import time
import math

from ezrassor_teleop_msgs.msg import TeleopAction
from ezrassor_teleop_msgs.msg import TeleopGoal
from ezrassor_teleop_msgs.msg import TeleopResult
from ezrassor_teleop_msgs.msg import TeleopFeedback

from gazebo_msgs.msg import LinkStates
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from tf import transformations

ALL_STOP = Twist()


class TeleopActionServer:
    def __init__(self):

        # ROS Actionlib Library
        self._server = actionlib.SimpleActionServer(
            "/teleop_action_server",
            TeleopAction,
            execute_cb=self.on_goal,
            auto_start=False,
        )

        self._server.start()

        # Set up publishers
        self.wheel_instructions = rospy.Publisher(
            "wheel_instructions", Twist, queue_size=1
        )
        self.front_arm_instructions = rospy.Publisher(
            "front_arm_instructions", Float32, queue_size=1
        )
        self.back_arm_instructions = rospy.Publisher(
            "back_arm_instructions", Float32, queue_size=1
        )
        self.front_drum_instructions = rospy.Publisher(
            "front_drum_instructions", Float32, queue_size=1
        )
        self.back_drum_instructions = rospy.Publisher(
            "back_drum_instructions", Float32, queue_size=1
        )

        # Make sure there are subscribers on the other end
        time.sleep(5)
        self.wheel_instructions.publish(ALL_STOP)
        self.front_arm_instructions.publish(0.0)
        self.back_arm_instructions.publish(0.0)
        self.front_drum_instructions.publish(0.0)
        self.back_drum_instructions.publish(0.0)

        # Subscribe to the link states so we can get the xy position from Gazebo
        rospy.Subscriber(
            "/gazebo/link_states", LinkStates, self.gazebo_state_callback
        )

        # Init the timer counter
        self._counter = 0

        # Give a success message!
        rospy.loginfo("EZRASSOR Teleop Action Server has been started")

    def gazebo_state_callback(self, data):
        """Saves the Gazebo link state (position) data for use in the server.
        Adapted from the ai_objects.py implementation"""

        self.x = 0
        self.y = 0
        self.heading = ""

        # Identify the index containing the position link state
        index = 0
        namespace = rospy.get_namespace()
        namespace = namespace[1:-1] + "::base_link"

        try:
            index = data.name.index(namespace)
        except Exception:
            rospy.logdebug("Failed to get index. Skipping...")
            return

        # Extract the information
        self.x = data.pose[index].position.x
        self.y = data.pose[index].position.y
        heading = self.quaternion_to_yaw(data.pose[index]) * 180 / math.pi

        if heading > 0:
            self.heading = heading
        else:
            self.heading = 360 + heading

    def quaternion_to_yaw(self, pose):
        """Helper function since Gazebo returns the forward kinematics of the robot
        as a quaternion"""
        quaternion = (
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        )
        euler = transformations.euler_from_quaternion(quaternion)
        return euler[2]

    def on_goal(self, goal):
        """Define all of the scenarios for handling a new goal from an action client."""

        # Python2 way to enforce type checking
        if not isinstance(goal, TeleopGoal):
            result = TeleopResult()
            rospy.logerr("Unknown goal received")
            self._server.set_aborted(result)  # Returns failure
            return

        msg = Twist()

        operation = goal.operation
        duration = goal.duration

        rospy.loginfo("New goal received!")
        rospy.loginfo("Action: ")
        rospy.loginfo(operation)

        rospy.loginfo("Duration: ")
        rospy.loginfo(duration)

        if operation == TeleopGoal.MOVE_FORWARD_OPERATION:
            msg.linear.x = 1
            self.wheel_instructions.publish(msg)

        elif operation == TeleopGoal.MOVE_BACKWARD_OPERATION:
            msg.linear.x = -1
            self.wheel_instructions.publish(msg)

        elif operation == TeleopGoal.ROTATE_LEFT_OPERATION:
            msg.angular.z = 1
            self.wheel_instructions.publish(msg)

        elif operation == TeleopGoal.ROTATE_RIGHT_OPERATION:
            msg.angular.z = -1
            self.wheel_instructions.publish(msg)

        elif operation == TeleopGoal.RAISE_FRONT_ARM_OPERATION:
            self.front_arm_instructions.publish(1.0)

        elif operation == TeleopGoal.LOWER_FRONT_ARM_OPERATION:
            self.front_arm_instructions.publish(-1.0)

        elif operation == TeleopGoal.RAISE_BACK_ARM_OPERATION:
            self.back_arm_instructions.publish(1.0)

        elif operation == TeleopGoal.LOWER_BACK_ARM_OPERATION:
            self.back_arm_instructions.publish(-1.0)

        elif operation == TeleopGoal.DIG_FRONT_DRUM_OPERATION:
            self.front_drum_instructions.publish(1.0)

        elif operation == TeleopGoal.DUMP_FRONT_DRUM_OPERATION:
            self.front_drum_instructions.publish(-1.0)

        elif operation == TeleopGoal.DIG_BACK_DRUM_OPERATION:
            self.back_drum_instructions.publish(1.0)

        elif operation == TeleopGoal.DUMP_BACK_DRUM_OPERATION:
            self.back_drum_instructions.publish(-1.0)

        else:
            # Otherwise, we must be stopping
            self.wheel_instructions.publish(ALL_STOP)
            self.front_arm_instructions.publish(0.0)
            self.back_arm_instructions.publish(0.0)
            self.front_drum_instructions.publish(0.0)
            self.back_drum_instructions.publish(0.0)

        # Time counter
        self._counter = 0

        success = False
        preempted = False

        # Feedback
        while not rospy.is_shutdown():
            self._counter += 1

            rospy.loginfo("Counter: ")
            rospy.loginfo(self._counter)

            rospy.loginfo("Duration: ")
            rospy.loginfo(duration)

            # If the client asks for a cancel
            if self._server.is_preempt_requested():
                preempted = True
                break

            # Success condition
            if self._counter > duration:
                success = True
                break

            feedback = TeleopFeedback()

            feedback.x = self.x
            feedback.y = self.y
            feedback.heading = str("{} degrees".format(self.heading))

            self._server.publish_feedback(feedback)
            time.sleep(1)

        # Stop the robot after every action
        self.wheel_instructions.publish(ALL_STOP)
        self.front_arm_instructions.publish(0.0)
        self.back_arm_instructions.publish(0.0)
        self.front_drum_instructions.publish(0.0)
        self.back_drum_instructions.publish(0.0)

        # Result
        result = TeleopResult()
        result.x = self.x
        result.y = self.y

        # Preempted means that a cancel command was issued from the client
        if preempted:
            rospy.loginfo("Preempted")
            self._server.set_preempted(result)
        # Return success result to the client
        elif success:
            rospy.loginfo("Success")
            self._server.set_succeeded(result)
        # Return failure result to the client
        else:
            rospy.loginfo("Failure")
            self._server.set_aborted(result)


def start_node():

    # Create a new action server node
    rospy.init_node("teleop_action_server")

    # Attach a new action server to the node
    TeleopActionServer()

    # Keep the node alive
    rospy.spin()
