import rospy

from std_msgs.msg import String
from std_msgs.msg import Int8
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import LinkStates
from sensor_msgs.msg import JointState, Imu, LaserScan
import actionlib

from ezrassor_swarm_control.msg import waypointAction, waypointResult
from ezrassor_swarm_control.srv import GetRoverStatus, GetRoverStatusResponse
# NOTE: Imported update status service
from ezrassor_swarm_control.srv import UpdateRoverStatus, UpdateRoverStatusResponse

import ai_objects as obj
import auto_functions as af
import utility_functions as uf


class RoverController:
    def __init__(
        self,
        target_x,
        target_y,
        start_x,
        start_y,
        movement_topic,
        front_arm_topic,
        back_arm_topic,
        front_drum_topic,
        back_drum_topic,
        obstacle_threshold,
        obstacle_buffer,
        move_increment,
        max_linear_velocity,
        max_angular_velocity,
        real_odometry,
        swarm_control,
    ):

        self.namespace = rospy.get_namespace()

        # Create Utility Objects
        self.world_state = obj.WorldState()
        self.ros_util = obj.ROSUtility(
            movement_topic,
            front_arm_topic,
            back_arm_topic,
            front_drum_topic,
            back_drum_topic,
            max_linear_velocity,
            max_angular_velocity,
            obstacle_threshold,
            obstacle_buffer,
            move_increment,
        )

        # Setup Subscriber Callbacks
        if real_odometry:
            # Get initial spawn coords
            self.world_state.initial_spawn(start_x, start_y)

            rospy.Subscriber(
                "odometry/filtered", Odometry, self.world_state.odometryCallBack
            )
        else:
            rospy.Subscriber(
                "/gazebo/link_states",
                LinkStates,
                self.world_state.simStateCallBack,
            )

        rospy.Subscriber("imu", Imu, self.world_state.imuCallBack)
        rospy.Subscriber(
            "joint_states", JointState, self.world_state.jointCallBack
        )
        rospy.Subscriber(
            "autonomous_toggles", Int8, self.ros_util.autoCommandCallBack
        )
        rospy.Subscriber(
            "obstacle_detection/combined",
            LaserScan,
            uf.on_scan_update,
            queue_size=1,
        )

        if swarm_control:
            self.ros_util.auto_function_command = 16

            # Create waypoint action server used to control the rover via the
            # swarm_controller
            self.server_name = "waypoint"
            self.waypoint_server = actionlib.SimpleActionServer(
                self.server_name,
                waypointAction,
                execute_cb=self.execute_action,
                auto_start=False,
            )

            self.waypoint_server.start()

            self.waypoint_server.register_preempt_callback(self.preempt_cb)

            rospy.loginfo("Rover waypoint server initialized.")

            # Register GetRoverStatus, used by the swarm controller to retrieve
            # a rover's position and battery
            self.status_service = rospy.Service(
                "rover_status", GetRoverStatus, self.send_status
            )
            rospy.loginfo("Rover status service initialized.")

            # NOTE: Added new 'update_status_service'
            # Register UpdateRoverStatus, used by the swarm controller to retrieve
            # a rover's position and battery
            self.update_status_service = rospy.Service(
                "update_rover_status", UpdateRoverStatus, self.update_status
            )
            rospy.loginfo("UPDATE rover status service initialized.")

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

    def send_status(self, request):
        """Send the rover's battery and pose to the swarm controller."""

        status = GetRoverStatusResponse()
        status.pose.position.x = self.world_state.positionX
        status.pose.position.y = self.world_state.positionY
        status.pose.position.z = self.world_state.positionZ
        status.battery = max(int(self.world_state.battery), 0)
        status.activity = self.world_state.activity
        status.digsite = self.world_state.assigned_digsite
        return status

    # NOTE: Created function to update rover activity 
    def update_status(self, request):
        """Send the rover's updated activity to the the swarm controller"""

        response = UpdateRoverStatusResponse(request.new_activity, request.assigned_digsite)
        self.world_state.activity = response.new_activity
        response.new_activity = self.world_state.activity

        self.world_state.assigned_digsite = response.assigned_digsite
        response.assigned_digsite = self.world_state.assigned_digsite
        # response.pose.position.x = self.world_state.positionX
        # response.pose.position.y = self.world_state.positionY
        # response.pose.position.z = self.world_state.positionZ
        # response.battery = max(int(self.world_state.battery), 0)
        return response
        


    def execute_action(self, goal):
        """Handle a swarm controller action."""

        if goal.target.x == -999 and goal.target.y == -999:
            rospy.loginfo(
                "Waypoint server {} executing charge command".format(
                    self.namespace + self.server_name
                )
            )

            # Set rover to charge
            af.charge_battery(self.world_state, self.ros_util)
            result = uf.build_result(self.world_state, preempted=0)
            self.waypoint_server.set_succeeded(result)

        elif goal.target.x == -998 and goal.target.y == -998:
            rospy.loginfo(
                "Waypoint server {} executing dig command".format(
                    self.namespace + self.server_name
                )
            )

            # Set rover to dig for 1000 seconds
            feedback, preempted = af.auto_dig(
                self.world_state, self.ros_util, 1000, self.waypoint_server
            )

            if (
                feedback is not None
                and not preempted
                and not self.waypoint_server.is_preempt_requested()
            ):
                result = uf.build_result(self.world_state, preempted=0)
                self.waypoint_server.set_succeeded(result)

        else:
            self.world_state.target_location = goal.target
            rospy.loginfo(
                "Waypoint server {} moving rover to {}".format(
                    self.namespace + self.server_name,
                    (goal.target.x, goal.target.y),
                )
            )

            # Set rover to autonomously navigate to target
            feedback, preempted = af.auto_drive_location(
                self.world_state, self.ros_util, self.waypoint_server
            )

            # Send resulting state to client and set server to succeeded, as
            # long as request wasn't preempted
            if (
                feedback is not None
                and not preempted
                and not self.waypoint_server.is_preempt_requested()
            ):
                result = waypointResult(feedback.pose, feedback.battery, 0)
                self.waypoint_server.set_succeeded(result)

    def preempt_cb(self):
        """Handle preempt request from swarm controller."""
        if self.waypoint_server.is_preempt_requested():
            # Stop the rover
            self.ros_util.publish_actions("stop", 0, 0, 0, 0)

            # Send rover status while preempting the waypoint goal
            result = uf.build_result(self.world_state, preempted=1)
            self.waypoint_server.set_preempted(result)

    def full_autonomy(self, world_state, ros_util):
        """ Full Autonomy Loop Function """

        rospy.loginfo("Full autonomy activated.")

        while ros_util.auto_function_command == 16:
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

        while True:
            while (
                ros_util.auto_function_command == 0
                or ros_util.auto_function_command == 32
            ):
                ros_util.publish_actions("stop", 0, 0, 0, 0)
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
                rospy.loginfo(
                    "Invalid auto-function request: %s.",
                    str(ros_util.auto_function_command),
                )

            ros_util.auto_function_command = 0
            ros_util.publish_actions("stop", 0, 0, 0, 0)
            ros_util.control_pub.publish(False)


def on_start_up(
    target_x,
    target_y,
    start_x,
    start_y,
    movement_topic,
    front_arm_topic,
    back_arm_topic,
    front_drum_topic,
    back_drum_topic,
    obstacle_threshold,
    obstacle_buffer,
    move_increment,
    max_linear_velocity=1,
    max_angular_velocity=1,
    real_odometry=False,
    swarm_control=False,
):
    """ Initialization Function  """

    # ROS Node Init Parameters
    rospy.init_node("autonomous_control")

    rover_controller = RoverController(
        target_x,
        target_y,
        start_x,
        start_y,
        movement_topic,
        front_arm_topic,
        back_arm_topic,
        front_drum_topic,
        back_drum_topic,
        obstacle_threshold,
        obstacle_buffer,
        move_increment,
        max_linear_velocity,
        max_angular_velocity,
        real_odometry,
        swarm_control,
    )

    # Start autonomous control loop if rover isn't being controlled by a swarm
    # controller
    if not swarm_control:
        rover_controller.autonomous_control_loop(
            rover_controller.world_state, rover_controller.ros_util
        )

        rospy.loginfo("Autonomous control initialized.")
