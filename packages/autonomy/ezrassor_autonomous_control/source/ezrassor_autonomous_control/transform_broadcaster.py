import rospy
import tf


def run():
	rospy.init_node('transform_broadcaster')
	camera_left = tf.TransformBroadcaster()
	camera_depth = tf.TransformBroadcaster()
        rospy.loginfo("Transform broadcaster initialized.")
	rate = rospy.Rate(100000.0)
	while not rospy.is_shutdown():
		camera_left.sendTransform((.3, 0.035, -.1), (-0.5, 0.5, -0.5, 0.5), rospy.Time.now(), "left_camera_optical_frame","base_link")
		camera_depth.sendTransform((.3, 0.035, -.1), (0.0, 0.0, 0.0, 1.0), rospy.Time.now(), "camera_depth_frame","base_link")
		
		rate.sleep()
