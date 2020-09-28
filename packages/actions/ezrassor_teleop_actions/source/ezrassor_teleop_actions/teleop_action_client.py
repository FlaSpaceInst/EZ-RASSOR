from ezrassor_teleop_msgs.msg import TeleopAction
from ezrassor_teleop_msgs.msg import TeleopGoal

import actionlib
import rospy
import time


class TeleopActionClient:
    def __init__(self):

        rospy.init_node("teleop_action_client")

        # ROS Actionlib Library
        self._client = actionlib.SimpleActionClient(
            "teleop_action_server", TeleopAction
        )

        # wait_for_server() occasionally fails instantly despite being given
        # a timeout. To prevent these failures we briefly sleep before waiting.
        # Hopefully this will be fixed in ROS 2.
        time.sleep(1)

        connected = self._client.wait_for_server(timeout=rospy.Duration(5))
        if not connected:
            rospy.logerr("Unable to connect to Teleop Action Server.")
            rospy.logerr(
                "Ensure the action server is running and"
                + " you have the ROS_NAMESPACE env var set"
                + " to the namespace of your action server."
            )
            return
        rospy.loginfo("Connected to Teleop Action Server.")

    def read_instructions(self, instructions_file):
        """Parse in a text file to identify new goals.
        Lines that start with # are considered comments and are ignored."""
        actions = []
        with open(instructions_file, "r") as reader:
            for line in reader.readlines():
                if line.startswith("#"):
                    continue
                line = line.rstrip()
                actions.append(line.lower())

        return actions

    def validate(self, action_list):
        """Check the list of parsed actions to ensure that they are all valid actions.
        If any single action is invalid, then the function will return False and the
        program will exit early."""

        if not type(action_list) is list:
            rospy.logerr(
                "Teleop Client script was not given an object of type list"
            )
            return False

        valid_operations = [
            TeleopGoal.MOVE_FORWARD_OPERATION,
            TeleopGoal.MOVE_BACKWARD_OPERATION,
            TeleopGoal.ROTATE_LEFT_OPERATION,
            TeleopGoal.ROTATE_RIGHT_OPERATION,
            TeleopGoal.RAISE_FRONT_ARM_OPERATION,
            TeleopGoal.LOWER_FRONT_ARM_OPERATION,
            TeleopGoal.RAISE_BACK_ARM_OPERATION,
            TeleopGoal.LOWER_BACK_ARM_OPERATION,
            TeleopGoal.DUMP_FRONT_DRUM_OPERATION,
            TeleopGoal.DIG_FRONT_DRUM_OPERATION,
            TeleopGoal.DUMP_BACK_DRUM_OPERATION,
            TeleopGoal.DIG_BACK_DRUM_OPERATION,
            TeleopGoal.STOP_OPERATION,
        ]

        # Validate every single action
        for action in action_list:

            action_components = action.split(" ")

            if len(action_components) != 2:
                rospy.logerr(
                    "Incorrect number of instructions given. Expected 2 got %s\n%s",
                    len(action_components),
                    action,
                )
                return False

            op = action_components[0]
            duration = action_components[1]

            if op not in valid_operations:
                rospy.logerr("Invalid operation received: %s", action)
                return False

            # Since Python2 does not support isnumeric, check for cast exceptions
            try:
                float(duration)
            except ValueError:
                rospy.logerr(
                    "Non-number passed in for time duration: \n%s", action
                )

        return True

    def send_movement_goal(self, actions):
        """Accepts a list of actions to send to the action server.
        Feedback returned indicates the robot's current x, y, and heading."""
        for action in actions:
            instruction = action.split(" ")
            direction = instruction[0]
            duration = float(instruction[1])

            goal = TeleopGoal(direction, duration)
            self._client.send_goal(
                goal,
                done_cb=self.done_callback,
                feedback_cb=self.feedback_callback,
            )

            rospy.loginfo("%s has been sent.", direction)

            # Wait an additional 2 seconds to account for any potential lag
            success = self._client.wait_for_result(
                rospy.Duration(duration + 2.0)
            )
            if not success:
                rospy.logwarn("HOUSTON, WE HAVE A PROBLEM")
                break

            rospy.loginfo(self._client.get_result())

    def done_callback(self, status, result):
        """When the goal is marked as finished, log the info to console."""
        rospy.loginfo("Status: ")
        rospy.loginfo(str(status))

        rospy.loginfo("Result: ")
        rospy.loginfo(str(result))

    def feedback_callback(self, feedback):
        """Feedback comes formatted from the action server."""
        rospy.loginfo(feedback)
