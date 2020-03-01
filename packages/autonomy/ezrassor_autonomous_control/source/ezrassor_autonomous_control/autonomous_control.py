import rospy
import sys

from std_msgs.msg import Int8, Int16, String
from geometry_msgs.msg import Point, Pose
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import LinkStates
from sensor_msgs.msg import JointState
from sensor_msgs.msg import Imu

import actionlib
from ezrassor_swarm_control.msg import *

from numpy import random as r

import ai_objects as obj
import auto_functions as af
import utility_functions as uf
import nav_functions as nf


class RoverController:
    def __init__(self, target_x, target_y, movement_topic, front_arm_topic,
                 back_arm_topic, front_drum_topic, back_drum_topic,
                 max_linear_velocity, max_angular_velocity,
                 real_odometry, swarm_control):

        self.namespace = rospy.get_namespace()

        # Create Utility Objects
        self.world_state = obj.WorldState()
        self.ros_util = obj.ROSUtility(movement_topic,
                                       front_arm_topic,
                                       back_arm_topic,
                                       front_drum_topic,
                                       back_drum_topic,
                                       max_linear_velocity,
                                       max_angular_velocity)

        # Setup Subscriber Callbacks
        if real_odometry:
            # This topic will be changed to represent whatever
            # topic the odometry data is being published to
            rospy.Subscriber('stereo_odometer/odometry',
                             Odometry,
                             self.world_state.odometryCallBack)
        else:
            rospy.Subscriber('/gazebo/link_states',
                             LinkStates,
                             self.world_state.simStateCallBack)

        rospy.Subscriber('imu',
                         Imu,
                         self.world_state.imuCallBack)
        rospy.Subscriber('joint_states',
                         JointState,
                         self.world_state.jointCallBack)
        rospy.Subscriber('obstacle_detect',
                         Int8,
                         self.world_state.visionCallBack)
        rospy.Subscriber('autonomous_toggles',
                         Int8,
                         self.ros_util.autoCommandCallBack)


        if swarm_control:
            # Create action server used to control the rover via the swarm_controller
            self.action_name = 'waypoint'
            self.action_server = actionlib.SimpleActionServer(self.action_name, waypointAction,
                                                              execute_cb=self.move_rover, auto_start=False)
            self.action_server.start()

            rospy.loginfo('Rover waypoint action server initialized.')

        else:
            # Basic autonomous control using the autonomous control loop
            target_location = Point()
            temp = Point()

            target_location.x = target_x
            target_location.y = target_y

            temp.x = target_x
            temp.y = target_y

            self.world_state.target_location = target_location
            self.world_state.dig_site = temp


    def preempt_check(self):
        if self.action_server.is_preempt_requested():
            rospy.loginfo('%s: Preempted' % self.action_name)
            self.action_server.set_preempted()
            return True

    def send_feedback(self):
        # Get rover's current pose
        current_pose = Pose()
        current_pose.position.x = self.world_state.positionX
        current_pose.position.y = self.world_state.positionY
        current_pose.position.z = self.world_state.positionZ
        current_pose.orientation = self.world_state.orientation

        feedback = waypointFeedback(current_pose=current_pose)

        # Publish feedback (current pose)
        self.action_server.publish_feedback(feedback)
        return feedback

    def move_rover(self, goal):
        """
        Callback executed when the swarm controller sends a goal to a rover via the rover's action server
        """
        rate = rospy.Rate(10)

        rospy.loginfo('Action server {} moving rover to {}'.format(
            self.namespace + self.action_name, (goal.target.x, goal.target.y)))

        # Check that preempt has not been requested by the client
        if self.preempt_check():
            return

        self.world_state.target_location = goal.target
        feedback = waypointFeedback()

        # Set arms up for travel
        uf.set_front_arm_angle(self.world_state, self.ros_util, 1.3)
        uf.set_back_arm_angle(self.world_state, self.ros_util, 1.3)

        # Loop until location is reached
        while af.at_target(self.world_state, self.ros_util):

            if uf.self_check(self.world_state, self.ros_util) != 1:
                rospy.logdebug('Status check failed.')
                self.action_server.set_preempted()
                return

            # Get new heading angle relative to current heading as (0,0)
            new_heading = nf.calculate_heading(self.world_state, self.ros_util)
            angle_difference = nf.adjust_angle(self.world_state.heading, new_heading)

            direction = 'right' if angle_difference < 0 else 'left'

            # Adjust heading until it matches new heading
            while not ((new_heading - 5) < self.world_state.heading < (new_heading + 5)):
                self.ros_util.publish_actions(direction, 0, 0, 0, 0)
                self.ros_util.rate.sleep()

            # Avoid obstacles by turning left or right if warning flag is raised
            if self.world_state.warning_flag == 1:
                uf.dodge_right(self.world_state, self.ros_util)
            if self.world_state.warning_flag == 2:
                uf.dodge_left(self.world_state, self.ros_util)
            if self.world_state.warning_flag == 3:
                uf.reverse_turn(self.world_state, self.ros_util)
                rospy.loginfo('Avoiding detected obstacle...')

            # Otherwise go forward
            self.ros_util.publish_actions('forward', 0, 0, 0, 0)
            self.ros_util.rate.sleep()
            feedback = self.send_feedback()

        # Stop the rover
        self.ros_util.publish_actions('stop', 0, 0, 0, 0)

        # Publish result
        result = waypointResult()
        result.final_pose = feedback.current_pose
        rospy.loginfo('Action server {} succeeded. Destination reached!'.format(self.namespace + self.action_name))

        self.action_server.set_succeeded(result)

    def full_autonomy(self, world_state, ros_util):
        """ Full Autonomy Loop Function """

        rospy.loginfo('Full autonomy activated.')

        while (ros_util.auto_function_command == 16):
            af.auto_drive_location(world_state, ros_util)
            if ros_util.auto_function_command != 16:
                break
            af.auto_dig(world_state, ros_util, 7)
            if ros_util.auto_function_command != 16:
                break
            af.auto_dock(world_state, ros_util)
            if ros_util.auto_function_command != 16:
                break
            af.auto_dump(world_state, ros_util, 4)
            world_state.target_location.x = world_state.dig_site.x
            world_state.target_location.y = world_state.dig_site.y

        world_state.target_location.x = world_state.dig_site.x
        world_state.target_location.y = world_state.dig_site.y

    def autonomous_control_loop(self, world_state, ros_util):
        """ Control Auto Functions based on auto_function_command input. """

        while (True):
            while ros_util.auto_function_command == 0 or ros_util.auto_function_command == 32:
                ros_util.publish_actions('stop', 0, 0, 0, 0)
                ros_util.rate.sleep()

            ros_util.control_pub.publish(True)

            if ros_util.auto_function_command == 1:
                af.auto_drive_location(world_state, ros_util)
            elif ros_util.auto_function_command == 2:
                af.auto_dig(world_state, ros_util, 10)
            elif ros_util.auto_function_command == 4:
                af.auto_dump(world_state, ros_util, 4)
            elif ros_util.auto_function_command == 8:
                af.auto_dock(world_state, ros_util)
            elif ros_util.auto_function_command == 16:
                self.full_autonomy(world_state, ros_util)
            else:
                rospy.loginfo('Invalid auto-function request: %s.',
                              str(ros_util.auto_function_command))

            ros_util.auto_function_command = 0
            ros_util.publish_actions('stop', 0, 0, 0, 0)
            ros_util.control_pub.publish(False)


def on_start_up(target_x, target_y, movement_topic, front_arm_topic,
                back_arm_topic, front_drum_topic, back_drum_topic,
                max_linear_velocity=1, max_angular_velocity=1,
                real_odometry=False, swarm_control=False):
    """ Initialization Function  """

    # ROS Node Init Parameters 
    rospy.init_node('autonomous_control')

    rover_controller = RoverController(target_x, target_y, movement_topic, front_arm_topic,
                                       back_arm_topic, front_drum_topic, back_drum_topic,
                                       max_linear_velocity, max_angular_velocity,
                                       real_odometry, swarm_control)

    if not swarm_control:
        rover_controller.autonomous_control_loop(rover_controller.world_state,
                                                 rover_controller.ros_util)

        rospy.loginfo('Autonomous control initialized.')
