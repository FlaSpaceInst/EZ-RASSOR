#!/usr/bin/env python
import rospy
from std_msgs.msg import Int8, Int16

# This publisher must be global so it is created at start up of the node.
# Otherwise it is created at the first time data is recieved at won't publish it's first value. 
publisher = rospy.Publisher('ezmain_topic', Int16, queue_size = 10)

def callback(data):

	global publisher
	rate = rospy.Rate(600) # number of hz

	publisher.publish(data)
	rospy.loginfo(data)
	rate.sleep()

def main():

	rospy.init_node('ez_main', anonymous = True)
	rospy.Subscriber("control_topic", Int16, callback)
	rospy.spin()

if __name__ == '__main__':

	try:
		main()

	except rospy.ROSInterruptException:
		pass