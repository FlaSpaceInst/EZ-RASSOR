#!/usr/bin/env python
import rospy
import tf


if __name__ == "__main__":
	rospy.init_node('left_image_frame_broadcaster')
	br = tf.TransformBroadcaster()
	rate = rospy.Rate(10.0)
	while not rospy.is_shutdown():
		br.sendTransform((1.0, 0.035, 0.0), (-0.5, 0.5, -0.5, 0.5), rospy.Time.now(), "left_camera_optical_frame","base_link")
		rate.sleep()