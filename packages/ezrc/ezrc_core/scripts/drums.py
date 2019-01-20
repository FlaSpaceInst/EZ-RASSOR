#!/usr/bin/env python
import rospy
from std_msgs.msg import Int16

def callback(data):

	data_in = data.data
	
	mask = 0b000000001111
	data_in &= mask
	data_string = "Drums:\t {0:04b}".format(data_in)

	if data_in == 0b1000:
		data_string = data_string + " -> Front Drums Dig"

		# Insert motor functions

	elif data_in == 0b0100:
		data_string = data_string + " -> Front Drums Dump"

		# Insert motor functions

	elif data_in == 0b0010:
		data_string = data_string + " -> Back Drums Dig"

		# Insert motor functions

	elif data_in == 0b0001:
		data_string = data_string + " -> Back Drums Dump"

		# Insert motor functions

	elif data_in == 0b1010:
		data_string = data_string + " -> Both Drums Dig"

		# Insert motor functions

	elif data_in == 0b0101:
		data_string = data_string + " -> Both Drums Dump"

		# Insert motor functions

	elif data_in == 0b1001:
		data_string = data_string + " -> Front Drums Dig, Back Drums Dump"

		# Insert motor functions

	elif data_in == 0b0110:
		data_string = data_string + " -> Front Drums Dump, Back Drums Dig"

		# Insert motor functions

	else:
		data_string = data_string + " -> Stop"

		# Halt motor functions

	print(data_string)

def main():

	rospy.init_node('ez_arms', anonymous = True)
	rospy.Subscriber('ezmain_topic', Int16, callback)
	rospy.spin()

if __name__ == '__main__':

	try:
		main()

	except rospy.ROSInterruptException:
		pass