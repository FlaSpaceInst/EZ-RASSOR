import rospy
import tf
from nav_msgs.msg import Odometry

def odometry_callback(data):
	odom = tf.TransformBroadcaster()
	map = tf.TransformBroadcaster()
	rate = rospy.Rate(10.0)
	while not rospy.is_shutdown():
		position = data.pose.pose.postion
		orientation = data.pose.pose.orientation
		odom.sendTransform((position.x, position.y, position.z), (orientation.x, orientation.y, orientation.z, orientation.w), rospy.Time.now(), "base_link","odom")
		map.sendTransform((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0), rospy.Time.now(), "odom","map")
		
		rate.sleep()

def run():
	rospy.init_node('odometry_frame_broadcaster')
	sub = rospy.Subscriber('/stereo_odometer/odometry', Odometry, odometry_callback)
	