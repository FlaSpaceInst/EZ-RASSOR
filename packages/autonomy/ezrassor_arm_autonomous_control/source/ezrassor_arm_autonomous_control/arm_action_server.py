import rospy
import actionlib

from control_msgs.msg import FollowJointTrajectoryAction


class MoveArm:
    def __init__(self):
        self.server = actionlib.SimpleActionServer(
            "/paver_arm_controller/follow_joint_trajectory",
            FollowJointTrajectoryAction,
            self.execute,
            False,
        )
        self.server.start()

    def execute(self, goal):
        rospy.loginfo(goal)


def on_start():
    rospy.init_node("arm_action_server")
    #server = MoveArm()
    rospy.spin()
