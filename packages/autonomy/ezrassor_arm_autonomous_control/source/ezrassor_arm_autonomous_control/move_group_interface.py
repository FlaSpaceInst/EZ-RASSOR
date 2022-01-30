import sys
import copy
import rospy
import moveit_commander
import geometry_msgs.msg
import numpy as np
from math import pi
from std_msgs.msg import Float64MultiArray, Float32
import arm_utility_functions as auf
from tf.transformations import quaternion_from_euler


# Container class for the moveit python interface so
# that needed information is available after the subscriber callback
class MoveIt:

    # Init for the MoveIt class and takes a RobotCommander,
    # PlanningSceneInterface, and MoveGroupCommander all from moveit_commander
    def __init__(self, robot, scene, move_group, claw_topic):
        self.robot = robot
        self.scene = scene
        self.move_group = move_group
        self.claw_pub = rospy.Publisher(claw_topic, Float32, queue_size=10)
        self.paver_count = 0

        self.move_group.set_max_velocity_scaling_factor(1)
        self.move_group.set_max_acceleration_scaling_factor(1)

    # Utility function for getting information on the current state of the robot
    def RobotInfo(self):

        # Get the planning frame for the moveit group
        planning_frame = self.move_group.get_planning_frame()

        # Get the name of the end effector link
        eef_link = self.move_group.get_end_effector_link()

        # Get the names of all groups associated to the robot
        group_names = self.robot.get_group_names()

        # Get the current joint states of the robot
        current_state = self.robot.get_current_state()

        print("Planning Frame: ", planning_frame, "\n")
        print("End Effector Link: ", eef_link, "\n")
        print("Planning Groups: ", group_names, "\n")
        print("Current Robot State: ", current_state, "\n")

    # Utility function for getting the current pose of the arm
    def GetCurrentPose(self):
        return self.move_group.get_current_pose().pose

    def GetJointAngles(self):
        return self.move_group.get_current_joint_values()

    # Move the arm to a specific joint goal then return to the home position
    def MoveArmJointGoal(self, data):
        joint_goal = self.move_group.get_current_joint_values()
        joint_goal[0] += data.data[0]
        joint_goal[1] += data.data[1]
        joint_goal[2] += data.data[2]
        joint_goal[3] += data.data[3]
        joint_goal[4] += data.data[4]

        self.move_group.go(joint_goal, wait=True)

        self.move_group.stop()

        # self.MoveHomePose

    # Move the arm based on a small increment for use with
    # manual inputs (gamepad/keyboard)
    def MoveArmApp(self, data):
        current_pose = self.GetCurrentPose()
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.position.x = current_pose.position.x + data.data[0]
        pose_goal.position.y = current_pose.position.y + data.data[1]
        pose_goal.position.z = current_pose.position.z + data.data[2]
        rospy.loginfo(pose_goal)
        self.move_group.set_pose_target(pose_goal)
        plan = self.move_group.go(wait=True)
        rospy.loginfo(plan)
        self.move_group.stop()
        self.move_group.clear_pose_targets()
        # self.MoveArmJointGoal([0.0, 0.0, 0.0, 0.0, 0.6])

        self.MoveHomePose

    # Move the arm so that the end effector is in a specific position,
    # operate grabber, then return to home position
    def MoveArmAuto(self, data):
        pose_goal = geometry_msgs.msg.Pose()
        quant = quaternion_from_euler(0, 0, (pi / 4) - data.data[3])
        norm = np.linalg.norm(quant)
        quant /= norm
        pose_goal.orientation.w = quant[3]
        pose_goal.orientation.x = quant[0]
        pose_goal.orientation.y = quant[1]
        pose_goal.orientation.z = quant[2]
        pose_goal.position.x = data.data[0]
        pose_goal.position.y = data.data[1]
        pose_goal.position.z = data.data[2]
        self.move_group.set_pose_target(pose_goal)
        plan = self.move_group.go(wait=True)
        rospy.loginfo("Arm response: \n")
        rospy.loginfo(plan)
        self.move_group.clear_pose_targets()
        rospy.loginfo(self.GetJointAngles())

    # Move the arm along a cartesian path that first raises the arm,
    # moves along the xy-plane, then lowers before operating the grabber;
    # then return to the home position
    def MoveAutoPath(self, data):
        waypoints = []

        wpose = self.GetCurrentPose()
        # wpose.position.z = 0.3
        # waypoints.append(copy.deepcopy(wpose))

        wpose.position.x = data.data[0]
        wpose.position.y = data.data[1]
        waypoints.append(copy.deepcopy(wpose))

        quant = quaternion_from_euler(0, 0, (pi / 4) + data.data[3])
        norm = np.linalg.norm(quant)
        quant /= norm

        wpose.orientation.w = quant[3]
        wpose.orientation.x = quant[0]
        wpose.orientation.y = quant[1]
        wpose.orientation.z = quant[2]
        waypoints.append(copy.deepcopy(wpose))

        wpose.position.z = data.data[2]
        waypoints.append(copy.deepcopy(wpose))

        (plan, fraction) = self.move_group.compute_cartesian_path(
            waypoints, 0.01, 0.0
        )

        self.move_group.execute(plan, wait=True)

        if data.data[3] == 1.0:
            self.CloseGrabber
        else:
            self.OpenGrabber

        self.MoveHomePose

    # Grabber Utility functions
    def CloseGrabber(self):
        self.paver_count += 1
        self.claw_pub.publish(Float32(6.0))
        rospy.sleep(rospy.Duration(1))
        auf.AttachPaver("grabber", self.paver_count)
        # self.move_group.set_max_velocity_scaling_factor(0.10)
        # self.move_group.set_max_acceleration_scaling_factor(0.05)

    def OpenGrabber(self):
        self.claw_pub.publish(Float32(-10.0))
        rospy.sleep(rospy.Duration(0.5))
        auf.DetachPaver("grabber", self.paver_count)
        rospy.sleep(rospy.Duration(1))
        self.move_group.set_max_velocity_scaling_factor(1)
        self.move_group.set_max_acceleration_scaling_factor(1)

    def getArmOutOfWay(self):
        self.move_group.set_named_target("Pickup_First_Paver_Prep")
        self.move_group.go(wait=True)
        self.move_group.stop()

    def postPickup(self):
        # self.move_group.set_named_target("Pickup_First_Paver_After")
        self.move_group.set_named_target("Test")
        self.move_group.go(wait=True)
        self.move_group.stop()

    def PickupFirstPaverOffBack(self):
        self.move_group.set_named_target("Pickup_First_Paver_Prep")
        self.move_group.go(wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()
        self.move_group.set_named_target("Pickup_First_Paver_Exec")
        self.move_group.go(wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()

    # Move arm to home position utility function
    def MoveHomePose(self):
        self.move_group.set_named_target("Home")
        self.move_group.go(wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()

    # Handle the command obtained from the
    # paver_arm_controller_instructions_topic subscriber
    def HandleCommand(self, data):
        if data.data[4] == 1.0:
            self.MoveArmAuto(data)
            self.CloseGrabber()
            self.MoveHomePose()
        elif data.data[4] == 2.0:
            self.MoveArmAuto(data)
            # self.MoveAutoPath(data)
            self.OpenGrabber()
            # self.MoveHomePose()
        elif data.data[4] == 3.0:
            self.PickupFirstPaverOffBack()
            self.CloseGrabber()
            self.postPickup()
            self.MoveHomePose()
        elif data.data[4] == 4.0:
            self.getArmOutOfWay()
        elif data.data[4] == 5.0:
            self.MoveHomePose()
        else:
            self.MoveArmJointGoal(data)


# Node Initiate function
def on_start_up(
    paver_arm_controller_instructions_topic,
    paver_arm_claw_instructions_topic,
):
    try:
        rospy.init_node("move_group_interface", anonymous=True)
        moveit_commander.roscpp_initialize(sys.argv)
        robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()
        group_name = "moveit_arm_controller"
        move_group = moveit_commander.MoveGroupCommander(group_name)

        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = "world"
        box_pose.pose.orientation.w = 1.0
        box_pose.pose.position.z = 0
        box_pose.pose.position.y = 0
        box_pose.pose.position.x = 0
        box_name = "ground"
        scene.add_box(box_name, box_pose, size=(3, 3, 0.1))

        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = "world"
        box_pose.pose.orientation.w = 1.0
        box_pose.pose.position.z = 1
        box_pose.pose.position.y = 0
        box_pose.pose.position.x = 0
        box_name = "ceiling"
        scene.add_box(box_name, box_pose, size=(3, 3, 0.1))

        move_it = MoveIt(
            robot, scene, move_group, paver_arm_claw_instructions_topic
        )

        # MultiArray Format: [x, y, z, flag]
        # flag: 1.0 = pickup, 2.0 = place, 3.0 = manual
        # Plan to substitute with custom message when time permits
        rospy.Subscriber(
            paver_arm_controller_instructions_topic,
            Float64MultiArray,
            move_it.HandleCommand,
        )
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
