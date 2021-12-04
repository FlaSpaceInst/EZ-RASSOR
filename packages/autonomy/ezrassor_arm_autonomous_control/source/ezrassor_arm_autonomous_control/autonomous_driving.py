import rospy
import roslaunch
import numpy as np
import math

from gazebo_msgs.msg import ModelStates, LinkStates
from rospy.exceptions import ROSInterruptException
from std_msgs.msg import Float64MultiArray, Int8, Float32
from sensor_msgs.msg import Imu, LaserScan, JointState, PointCloud2
from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from moveit_msgs.msg import MoveGroupActionResult
import control_msgs.msg

import ezrassor_autonomous_control.utility_functions as uf
import ezrassor_autonomous_control.ai_objects as ao
import ezrassor_autonomous_control.auto_functions as af
import ezrassor_autonomous_control.nav_functions as nf
import object_detection as od
import arm_utility_functions as auf

from tf.transformations import euler_from_quaternion

class ArmAutoHelper:
    # Init function for the class ArmAutoHelper that integrates the arm autonomus movement with the autonomous driving of the rover
    def __init__(self, target_x, target_y, start_x, start_y, movement_topic, front_arm_topic, back_arm_topic, front_drum_topic, back_drum_topic, obstacle_threshold, obstacle_buffer, move_increment, max_linear_velocity, max_angular_velocity):

        #uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        #roslaunch.configure_logging(uuid)

        self.namespace = rospy.get_namespace()

        self.paver_count = 0
        self.heading = None

        self.target_x = target_x
        self.target_y = target_y
        self.start_x = start_x
        self.start_y = start_y
        correct_move_topic = "/ezrassor1/" + movement_topic
        self.movement_topic = correct_move_topic
        self.front_arm_topic = front_arm_topic
        self.back_arm_topic = back_arm_topic
        self.front_drum_topic = front_drum_topic
        self.back_drum_topic = back_drum_topic
        self.obstacle_threshold = obstacle_threshold
        self.obstacle_buffer = obstacle_buffer
        self.move_increment = move_increment
        self.max_linear_velocity = max_linear_velocity
        self.max_angular_velocity = max_angular_velocity
        
        self.world_state = ao.WorldState()
        self.ros_util = ao.ROSUtility(
            self.movement_topic,
            self.front_arm_topic,
            self.back_arm_topic,
            self.front_drum_topic,
            self.back_drum_topic,
            self.max_linear_velocity,
            self.max_angular_velocity,
            self.obstacle_threshold,
            self.obstacle_buffer,
            self.move_increment,
        )

        self.ros_util.auto_function_command = 3
        self.ros_util.threshold = 0.2
        self.ros_util.move_increment = 0.1
        self.ros_util.obstacle_threshold = 2.0
        self.ros_util.obstacle_buffer = 0.01
        self.ros_util.max_linear_velocity = 1.0
        self.ros_util.max_angular_velocity = 1.0

        self.world_state.startPositionX = start_x
        self.world_state.startPositionY = start_y
        self.world_state.target_location = Point()
        self.world_state.target_location.x = target_x
        self.world_state.target_location.y = target_y
        self.paver_angle = None
        self.placement_flag = True
        self.placement_attempts = 0
        self.pickup_counter = 0
        self.arm_data = Float64MultiArray()
        self.paver_tracker = 1
        self.paver_tracker_cap = 3
        self.paver_tracker_start = 1
        self.pad_size = 3
        self.row_counter = 0
        self.paver_depth = 1.0
        self.detection_flag = True

        self.targets = []
        self.target_counter = 1
        for x in range(self.pad_size):
            for y in range(self.pad_size):
                self.targets.append((self.target_x-(x*0.355), self.target_y-(y*0.455)))


        new_start = [0,0]
        new_target = [0,0]
        last_paver = [0,0]

        self.arm_base_link_x = None
        self.arm_base_link_y = None
        self.model_poses = None
        self.launch = roslaunch.scriptapi.ROSLaunch()
        self.launch.start()

        rospy.Subscriber('/gazebo/model_states', ModelStates, self.setModelStates)
        rospy.Subscriber('/spawn_paver', Float64MultiArray, self.spawnPaver)
        #rospy.Subscriber('/gazebo/link_states', LinkStates, self.setLinkStates)
        #rospy.Subscriber('/demo_arm', Int16, self.executeDemo)
        #rospy.Subscriber('/check_model_state', Int16, self.checkModelState)

        rospy.Subscriber(
                "/gazebo/link_states",
                LinkStates,
                self.world_state.simStateCallBack,
            )
        rospy.Subscriber("imu", Imu, self.world_state.imuCallBack)
        rospy.Subscriber(
            "joint_states", JointState, self.world_state.jointCallBackV2
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

        rospy.Subscriber(
            'depth/points',
            PointCloud2,
            callback=None
        )

        rospy.Subscriber(
            'object_detection/paver_location',
            Float64MultiArray,
            self.objectLocationCallback
        )

        rospy.Subscriber("/ezrassor1/move_group/result", MoveGroupActionResult, self.armResponse)
        rospy.Subscriber("move_home", Float32, self.moveHomeCallback)
        

        #rospy.Subscriber(
        #    '/ground_truth/state',
        #    Odometry,
        #    self.setOdometry
        #)

        self.arm_pub = rospy.Publisher('/ezrassor1/paver_arm_controller_instructions', Float64MultiArray, queue_size=10)
        self.object_detection_pub = rospy.Publisher('/ezrassor1/paver_arm_object_detection_command_instructions', Float32, queue_size=10)

    # Callback funtion for the move_home publisher
    def moveHomeCallback(self, data): 
        self.returnToSpawnPrep([self.model_poses.position.x, self.model_poses.position.y])
        self.auto_drive_location(self.world_state, self.ros_util, waypoint_server=None)
        rospy.sleep(rospy.Duration(secs=0.5))

    # Callback for setting the model states
    def setOdometry (self, data):
        self.quant = [data.pose.pose.orientation.w, data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z]
        norm = np.linalg.norm(self.quant)
        self.quant /= norm

    # Arm movement function used to test connection between this node and the move group interface node
    def fullArmMove(self, target):
        msg = Float64MultiArray()
        msg.data = [0.0, 0.0, 0.0, 0.0, 3.0]
        self.arm_pub.publish(msg)
        rospy.sleep(rospy.Duration(secs=2))
        msg.data = [target[0], target[1], target[2], 0.0, 2.0]
        self.arm_pub.publish(msg)

    # Callback function for gettng the joint angles regarding the drum arms
    def jointCallBack (self, data):
        rospy.loginfo(data.position[0])
        self.world_state.jointCallBackV2(data)

    # Callback function for handling the response from the move group interface
    def armResponse (self, data):
        if data.status.status == 3:
            if self.arm_data.data == [0.0, 0.0, 0.0, 0.0, 3.0]:
                if self.pickup_counter < 3:
                    self.pickup_counter += 1
                else:
                    self.placement_flag = False
                    self.placement_attempts = 0
                    self.pickup_counter = 0
            #elif self.arm_data.data[4] == 2.0:
            #    self.placement_flag = False
            #    self.placement_attempts = 0
            #    rospy.sleep(rospy.Duration(secs=1))
            else:
                self.placement_flag = False
                self.placement_attempts = 0
        elif data.status.status == 4:
            if self.placement_attempts > 3:
                rospy.loginfo("Placement Failed")
                self.placement_flag = False
                self.placement_attempts = 0
            else:
                self.placement_attempts += 1
                self.arm_pub.publish(self.arm_data)
    
    # Callback function for setting the rover's model state
    def setModelStates(self, data):

        index = data.name.index('ezrassor1')

        self.model_names = data.name[index]
        self.model_poses = data.pose[index]
        self.model_twists = data.twist[index]

    # Callback function for setting the rover's link states
    def setLinkStates(self, data):
        index = data.name.index('link1')

        self.arm_base_link_x = round(data.pose[index].position.x, 5)
        self.arm_base_link_y = round(data.pose[index].position.y, 5)

    # Function to get the orientation of rover in radians
    def getRadOrientationOfRover (self):
        return euler_from_quaternion([self.model_poses.orientation.x, self.model_poses.orientation.y, self.model_poses.orientation.z, self.model_poses.orientation.w])

    # Function for converting degrees to radians
    def degToRad (self, deg):
        return math.radians(deg)

    # Function for keeping track of what paver model type should be spawned based on the current row count
    def paverIncrementer (self, count):
        if count%self.pad_size == 0:
            if self.paver_tracker_cap == 9:
                self.paver_tracker_cap = 3
                self.paver_tracker_start = 1
                self.paver_tracker = 1
            else:
                self.paver_tracker_cap += 3
                self.paver_tracker_start += 3
                self.paver_tracker = self.paver_tracker_start

    # Function for spawning a paver on the back of the rover
    def spawnPaver(self):
        self.paver_count += 1
        (roll, pitch, yaw) = self.getRadOrientationOfRover()
        #current_rpy = euler_from_quaternion(self.quant)
        rospy.loginfo(yaw)
        rospy.loginfo(yaw-1.54)
        data = [1, round(self.model_poses.position.x, 5)-0.07, round(self.model_poses.position.y, 5)+0.04, round(self.model_poses.position.z, 5) + 0.13, yaw-1.54]
        rospy.loginfo(data)
        #data = [data.data[0], ]
        package = 'gazebo_ros'
        node_type = 'spawn_model'
        output='screen'
        args= '-database paver_{} -sdf -model paver{} -x {} -y {} -z {} -Y {}'.format(self.paver_tracker, self.paver_count, data[1], data[2], data[3], data[4])
        if self.paver_tracker == self.paver_tracker_cap:
            self.paver_tracker = self.paver_tracker_start
        else:
            self.paver_tracker += 1
        node = roslaunch.core.Node(
            package=package,
            node_type=node_type,
            output=output,
            args=args
        )
        self.launch.launch(node)
    
    # Function for spawning a paver at a specific target
    def spawnPaverAtTarget (self, target):
        package = 'gazebo_ros'
        node_type = 'spawn_model'
        output='screen'
        args= '-database paver -sdf -model paver_test -x {} -y {} -z {} -Y {}'.format(target[0], target[1], target[2], 0)
        node = roslaunch.core.Node(
            package=package,
            node_type=node_type,
            output=output,
            args=args
        )
        self.launch.launch(node)

    # Function for checking the state of the model
    def checkModelState (self, data):
        rospy.loginfo(self.model_names)
        rospy.loginfo(self.model_poses)

    # Modified rover driving function to account for the missing front drum arm other aspects to adapt the solution to work with our arm
    def auto_drive_location(self, world_state, ros_util, waypoint_server=None):
        """ Navigate to location. Avoid obstacles while moving toward location. """

        # Action server will print it's own info
        if waypoint_server is None:
            rospy.loginfo(
                "Auto-driving to [%s, %s]...",
                str(world_state.target_location.x),
                str(world_state.target_location.y),
            )
        # Send feedback to waypoint client if being controlled by swarm controller
        preempted = False
        feedback = uf.send_feedback(world_state, waypoint_server)

        # Set arms up for travel
        # uf.set_front_arm_angle(world_state, ros_util, 1.3)
        uf.set_back_arm_angle(world_state, ros_util, 0.2)
        rospy.loginfo('Finished with drums')
        # Check rover battery, hardware, and if it's flipped over
        if uf.self_check(world_state, ros_util) != 1:
            preempted = True

            # Cancel action server request is self check failed
            if waypoint_server is not None:
                waypoint_server.set_preempted()

            rospy.logdebug("Status check failed.")
            return feedback, preempted

        # Before we head towards our goal, turn to face it.
        # Get new heading angle relative to current heading
        new_heading_degrees = nf.calculate_heading(world_state)
        angle2goal_radians = nf.adjust_angle(
            world_state.heading, new_heading_degrees
        )

        # If our angle is less than zero, then we would expect a right turn
        # otherwise turn left.
        if angle2goal_radians < 0:
            direction = "right"
        else:
            direction = "left"

        uf.turn(new_heading_degrees, direction, world_state, ros_util)
        ros_util.publish_actions("stop", 0, 0, 0, 0)

        if not af.at_target(
            world_state.positionX,
            world_state.positionY,
            world_state.target_location.x,
            world_state.target_location.y,
            ros_util.threshold,
        ):
           # Move towards the direction chosen.
            uf.move(ros_util.move_increment, world_state, ros_util)

            world_state.battery -= 0.1

            # Send feedback to waypoint action client
            feedback = uf.send_feedback(world_state, waypoint_server) 

        # Main loop until location is reached
        while not af.at_target(
            world_state.positionX,
            world_state.positionY,
            world_state.target_location.x,
            world_state.target_location.y,
            ros_util.threshold,
        ):

            # Check that the waypoint client request hasnt been canceled
            if (
                waypoint_server is not None
                and waypoint_server.is_preempt_requested()
            ):
                preempted = True
                break

            # Set arms up for travel
            # uf.set_front_arm_angle(world_state, ros_util, 1.3)
            uf.set_back_arm_angle(world_state, ros_util, 0.2)

            # Check rover battery, hardware, and if it's flipped over
            if uf.self_check(world_state, ros_util) != 1:
                preempted = True
                # Cancel waypoint client request is self check failed
                if waypoint_server is not None:
                    waypoint_server.set_preempted()

                rospy.logdebug("Status check failed.")
                break

            angle = uf.get_turn_angle(world_state, ros_util, flag=False)

            # If our angle is less than zero, then we would expect a right turn
            # otherwise turn left.
            direction = "right" if angle < 0 else "left"

            # Turn towards the direction chosen.
            uf.turn(
                nf.rel_to_abs(world_state.heading, angle),
                direction,
                world_state,
                ros_util,
            )

            # Move towards the direction chosen.
            uf.move(ros_util.move_increment, world_state, ros_util)

            world_state.battery -= 0.1

            # Send feedback to waypoint action client
            feedback = uf.send_feedback(world_state, waypoint_server)

        # Action server will print its own info
        if waypoint_server is None:
            rospy.loginfo("Destination reached!")

        ros_util.publish_actions("stop", 0, 0, 0, 0)
        return feedback, preempted

    # Function for calculating the current heading of the rover
    def calculate_heading(self, world_state, location):
        # Calculate the new heading of the robot given its current location and the
        # target location.
        y2 = location[0]
        y1 = world_state.positionY
        x2 = location[1]
        x1 = world_state.positionX

        # This is the x and y distances.
        dy = y2 - y1
        dx = x2 - x1

        # Get the angle in radians.
        new_heading = math.atan2(dy, dx)

        # Convert the angle to degrees.
        new_heading = 180 * new_heading / math.pi

        # Since gazebo only uses 0 through 360 degree, we must bound any negative
        # angles to positive ones.
        if new_heading < 0:
            new_heading = 360 + new_heading

        return new_heading

    # Function the utilizes the rover's autonomous driving functions to turn to a desired heading
    def turnToDesiredHeading (self, pass_angle):
        location = [self.world_state.positionY, self.world_state.positionX + 1]
        new_heading_degrees = self.calculate_heading(self.world_state, location)
        angle2goal_radians = nf.adjust_angle(
            self.world_state.heading, new_heading_degrees
        )

        # If our angle is less than zero, then we would expect a right turn
        # otherwise turn left.
        direction = "right" if angle2goal_radians < 0 else "left"

        #if angle == 0:
        #    if direction == "left":
        #        angle -= 0.2
        #    else:
        #        angle += 0.2

        # Turn towards the direction chosen.
        uf.turn(
            #nf.rel_to_abs(self.world_state.heading, angle),
            new_heading_degrees,
            direction,
            self.world_state,
            self.ros_util,
            )
        self.ros_util.publish_actions("stop", 0, 0, 0, 0)
        rospy.loginfo("End turning")
        rospy.loginfo("Current Heading: ")
        rospy.loginfo(self.world_state.heading)

    # Full autonomous function to perform the arm's demo routine
    def full_auto (self):

        # Spawn First Paver
        self.spawnPaver()
        rospy.sleep(rospy.Duration(secs=2))
        auf.AttachPaver('platform', self.paver_count)

        rospy.loginfo("Current Heading: ")
        rospy.loginfo(self.world_state.heading)

        #while self.ros_util.auto_function_command == 3:

        # Move arm out of the way of rover and camera for driving
        self.arm_data.data = [0.0, 0.0, 0.0, 1.0, 4.0]
        self.arm_pub.publish(self.arm_data)
        while self.placement_flag:
            rospy.sleep(1)
        self.placement_flag = True

        # Drive to first target location for placement
        self.auto_drive_location(self.world_state, self.ros_util, waypoint_server=None)
        #if self.ros_util.auto_function_command != 3:
        #    break
        rospy.sleep(rospy.Duration(secs=0.5))

        # Start auto loop
        while True:

            # Record heading of rover before first placement to remove orientation issues
            #if self.paver_angle == None:
            #    self.paver_angle = self.getRadOrientationOfRover()
            #    rospy.loginfo("Paver Angle: ")
            #    rospy.loginfo(self.paver_angle)

            # Turn to face same heading as first placement to ensure orientation of paver
            #else:
            #    rospy.loginfo('Turning for placement')
            #    self.turnToDesiredHeading(self.paver_angle[2])

            current_orientation = self.getRadOrientationOfRover()
            new_angle = (2*np.pi)-np.abs(current_orientation[2])
            self.turnToDesiredHeading(new_angle)
            rospy.sleep(rospy.Duration(secs=0.5))

            if self.paver_count > 1:
                rospy.loginfo("Publishing to Object Detection")
                self.object_detection_pub.publish(Float32(self.paver_count))
                while self.detection_flag:
                    rospy.sleep(1)
                self.detection_flag = True

                temp_flag = False
                while temp_flag:
                    if self.paver_depth > 1.0:
                        twist_message = Twist()
                        twist_message.linear.x = 0.1
                        self.ros_util.movement_pub.publish(twist_message)
                        rospy.sleep(rospy.Duration(secs=0.5))
                        self.ros_util.publish_actions("stop", 0, 0, 0, 0)

                        self.object_detection_pub.publish(Float32(1))
                        while self.detection_flag:
                            rospy.sleep(1)
                        self.detection_flag = True
                    elif self.paver_depth < 0.5:
                        twist_message = Twist()
                        twist_message.linear.x = -0.1
                        self.ros_util.movement_pub.publish(twist_message)
                        rospy.sleep(rospy.Duration(secs=0.5))
                        self.ros_util.publish_actions("stop", 0, 0, 0, 0)

                        self.object_detection_pub.publish(Float32(1))
                        while self.detection_flag:
                            rospy.sleep(1)
                        self.detection_flag = True
                    else:
                        temp_flag = False

            rospy.loginfo("Paver Depth: ")
            rospy.loginfo(self.paver_depth)
            auf.DetachPaver('platform', self.paver_count)

            # Pick up paver
            self.arm_data.data = [0.0, 0.0, 0.0, 0.0, 3.0]
            self.arm_pub.publish(self.arm_data)
            while self.placement_flag:
                rospy.sleep(1)
            self.placement_flag = True
            rospy.loginfo("End first sleep")

            # Place paver
            temp = 0
            temp_depth = 0
            if self.world_state.heading > 180:
                temp = -(360-self.world_state.heading)
            else:
                temp = self.world_state.heading
            if self.paver_count > self.pad_size:
                temp_depth = self.row_paver_depth
            else:
                temp_depth = self.paver_depth
            self.arm_data.data = [temp_depth, 0.25, 0.23, self.degToRad(temp), 2.0]
            self.arm_pub.publish(self.arm_data)
            while self.placement_flag:
                rospy.sleep(1)
            self.placement_flag = True
            rospy.loginfo("End second sleep")

            # Move arm back to standby for driving
            self.arm_data.data = [0.0, 0.0, 0.0, 1.0, 4.0]
            self.arm_pub.publish(self.arm_data)
            while self.placement_flag:
                rospy.sleep(1)
            self.placement_flag = True
            rospy.loginfo("End third sleep")

            # Record the location of the last placed paver
            self.last_paver = [self.model_poses.position.x + 1.0, self.model_poses.position.y + 0.3]

            # Return to spawn to get new paver
            self.returnToSpawnPrep([self.model_poses.position.x, self.model_poses.position.y])
            self.auto_drive_location(self.world_state, self.ros_util, waypoint_server=None)
            rospy.sleep(rospy.Duration(secs=0.5))

            # Set heading to spawn heading to ensure placement of paver
            current_orientation = self.getRadOrientationOfRover()
            new_angle = (2*np.pi)-np.abs(current_orientation[2])
            self.turnToDesiredHeading(new_angle)
            rospy.sleep(rospy.Duration(secs=0.5))
            
            # Spawn next paver
            self.spawnPaver()
            rospy.sleep(rospy.Duration(secs=2))
            auf.AttachPaver('platform', self.paver_count)

            # Set next placement location and drive there
            #new_target = [self.last_paver[0] - 1.0, self.last_paver[1] - 0.5]
            #new_target = [5, self.last_paver[1]-0.805]
            if self.target_counter == self.pad_size**2:
                rospy.loginfo("Pad complete")
                break
            else:
                new_target = self.targets[self.target_counter]
                self.target_counter += 1
                self.paverIncrementer(self.target_counter)
            self.returnToPaverPlace(new_target)

    # Function for preparing the rover to return to its spawn location for paver pickup
    def returnToSpawnPrep (self, current_location):
        self.world_state.startPositionX = current_location[0]
        self.world_state.startPositionY = current_location[1]
        self.world_state.target_location = Point()
        self.world_state.target_location.x = self.start_x
        self.world_state.target_location.y = self.start_y

    # Function for preparing the rover to travel to the next location for paver placement
    def returnToPaverPlace (self, target_location):
        self.world_state.startPositionX = self.start_x
        self.world_state.startPositionY = self.start_y
        self.world_state.target_location = Point()
        self.world_state.target_location.x = self.start_x
        self.world_state.target_location.y = target_location[1]
        self.auto_drive_location(self.world_state, self.ros_util, waypoint_server=None)
        self.world_state.startPositionX = self.start_x
        self.world_state.startPositionY = target_location[1]
        self.world_state.target_location = Point()
        self.world_state.target_location.x = target_location[0]
        self.world_state.target_location.y = target_location[1]
        self.auto_drive_location(self.world_state, self.ros_util, waypoint_server=None)

    # Object detection callback function to use the information obtained by object detection to get the location of the last paver in relation to the rover
    def objectLocationCallback (self, data):
        depth = np.sqrt((data.data[2]**2)-(0.23**2))
        if self.paver_count == self.pad_size+1:
            self.paver_depth = depth-0.355
            self.row_paver_depth = depth-0.355+0.4878
        else:
            self.paver_depth = depth
        self.paver_depth += 0.4878
        self.detection_flag = False

    # Visual network testing function to ensure connectivity
    def visualNetworkTest (self):
        #arm_data = Float64MultiArray()
        #arm_data.data = [1.0, 0.3, 0.2, 1.0, 4.0]
        #self.arm_pub.publish(arm_data)
        self.spawnPaverAtTarget([2.0, 0.3, 0.1])
        rospy.sleep(rospy.Duration(secs=2))
        self.object_detection_pub.publish(1.0)

# Start up function for the autonomous driving node
def onStart (
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
    try:
        rospy.init_node('autonomous_driving')
        driver = ArmAutoHelper(
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
        )

        rospy.sleep(rospy.Duration(secs=2))
        #driver.armTest()
        #driver.visualNetworkTest()
        #driver.full_auto()


    except ROSInterruptException:
        pass