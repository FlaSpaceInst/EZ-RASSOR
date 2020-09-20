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
QUEUE_SIZE = 10


class TeleopActionServer:
    def __init__(self):

        # Create a new action server node
        rospy.init_node("teleop_action_server")

        # Initial assumption is that the server is not currently executing any operation
        self.executing_goal = False

        # ROS Actionlib Library
        self._server = actionlib.SimpleActionServer(
            "teleop_action_server",
            TeleopAction,
            execute_cb=self.on_goal,
            auto_start=False,
        )

        # Set up publishers
        self.wheel_instructions = rospy.Publisher(
            rospy.get_param(rospy.get_name() + "/wheel_instructions_topic"),
            Twist,
            queue_size=QUEUE_SIZE,
        )
        self.front_arm_instructions = rospy.Publisher(
            rospy.get_param(rospy.get_name() + "/front_arm_instructions_topic"),
            Float32,
            queue_size=QUEUE_SIZE,
        )
        self.back_arm_instructions = rospy.Publisher(
            rospy.get_param(rospy.get_name() + "/back_arm_instructions_topic"),
            Float32,
            queue_size=QUEUE_SIZE,
        )
        self.front_drum_instructions = rospy.Publisher(
            rospy.get_param(
                rospy.get_name() + "/front_drum_instructions_topic"
            ),
            Float32,
            queue_size=QUEUE_SIZE,
        )
        self.back_drum_instructions = rospy.Publisher(
            rospy.get_param(rospy.get_name() + "/back_drum_instructions_topic"),
            Float32,
            queue_size=QUEUE_SIZE,
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

        # Initialize positional/state data.
        self.x = 0
        self.y = 0
        self.heading = ""

        # Give a success message!
        rospy.loginfo("EZRASSOR Teleop Action Server has been initialized")

    def spin(self):
        """Start the node"""
        self._server.start()
        rospy.spin()

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

    def execution_timer_callback(self, event):
        """If this callback is called, it is to stop execution"""
        self.executing_goal = False

    def on_goal(self, goal):
        """Define all of the scenarios for handling a new goal from an action client."""

        # Python2 way to enforce type checking
        if not isinstance(goal, TeleopGoal):
            result = TeleopResult()
            rospy.logerr("Unknown goal received")
            self._server.set_aborted(result)  # Returns failure
            return

        self.executing_goal = True

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

        # Start new timer for operation
        rospy.Timer(
            rospy.Duration(duration),
            self.execution_timer_callback,
            oneshot=True,
        )
        t0 = time.time()

        # Feedback
        while not rospy.is_shutdown() and self.executing_goal:

            elapsed = time.time() - t0
            rospy.loginfo("Elapsed (seconds): ")
            rospy.loginfo(elapsed)

            rospy.loginfo("Duration: ")
            rospy.loginfo(duration)

            if self._server.is_preempt_requested():
                rospy.loginfo("Preempted")
                self.executing_goal = False
                self._server.set_preempted(result)
                return

            feedback = TeleopFeedback()
            feedback.x = self.x
            feedback.y = self.y
            feedback.heading = str("{} degrees".format(self.heading))
            try:
                self._server.publish_feedback(feedback)
            except rospy.ROSException:
                self._server.set_aborted(
                    None, text="Unable to publish feedback. Has ROS stopped?"
                )
                return

        try:
            # Stop the robot after every action
            self.wheel_instructions.publish(ALL_STOP)
            self.front_arm_instructions.publish(0.0)
            self.back_arm_instructions.publish(0.0)
            self.front_drum_instructions.publish(0.0)
            self.back_drum_instructions.publish(0.0)
        except rospy.ROSException:
            self._server.set_aborted(
                None, text="Unable to publish all stop. Has ROS stopped?"
            )
            return

        # Result
        result = TeleopResult()
        result.x = self.x
        result.y = self.y

        # Return success result to the client
        rospy.loginfo("Success")
        self._server.set_succeeded(result)
