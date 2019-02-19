#!/usr/bin/env python
import rospy
from std_msgs.msg import Int8, Int16
from sensor_msgs.msg import Joy

# Global so topic initiated at start of node
publisher = rospy.Publisher('ez_main_topic', Int16, queue_size = 10)

def callback(data):

	global publisher

	# Raw controller input data indexes

	# data.buttons[index]
	# 0 A : Back Drum Dump
	# 1 B : Front Drum Dump
	# 2 X : Back Drum Dig
	# 3 Y : Front Drum Dig
	# 4 LB : Back Arm Up
	# 5 RB : Front Arm Up
	# 6 back : NA
	# 7 start : Supervisor Mode
	# 8 power : NA
	# 9 Button stick left : NA
	# 10 Button stick right : NA

	# data.axes[index]
	# 0 Left/Right Axis stick left : Turn Left/Right
	# 1 Up/Down Axis stick left : Move Forward/Reverse
	# 2 LT : Back Arm Down
	# 3 Left/Right Axis stick right : NA
	# 4 Up/Down Axis stick right : NA
	# 5 RT : Front Arm Down
	# 6 cross key left/right : Function 2/3
	# 7 cross key up/down : Function 1/4

	data_out = 0b000000000000

	# Wheels

	# F B L R
	# 0 0 0 0

	data_out <<= 4

	if data.axes[1] > 0.5:
		data_out |= 0b1000
		
	elif data.axes[1] < -0.5:
		data_out |= 0b0100

	elif data.axes[0] > 0.5:
		data_out |= 0b0010
		
	elif data.axes[0] < -0.5:
		data_out |= 0b0001

	# Arms

	# FAU FAD BAU BAD
	#  0   0   0   0

	data_out <<= 4

	if data.buttons[5] > 0.5:
		data_out |= 0b1000

	if data.axes[5] < -0.5:
		data_out |= 0b0100

	if data.buttons[4] > 0.5:
		data_out |= 0b0010

	if data.axes[2] < -0.5:
		data_out |= 0b0001


	# Drums

	# FDG FDP BDG BDP
	#  0   0   0   0

	data_out <<= 4

	if data.buttons[3] > 0.5:
		data_out |= 0b1000

	if data.buttons[1] > 0.5:
		data_out |= 0b0100

	if data.buttons[2] > 0.5:
		data_out |= 0b0010

	if data.buttons[0] > 0.5:
		data_out |= 0b0001

	publisher.publish(data_out)
	rospy.loginfo("Controller: {0:012b}".format(data_out))

	
def main():

	print("Controller node started")
	global publisher
	rospy.init_node('ez_controller', anonymous = True)
	rate = rospy.Rate(600) # number of hz
	
	# Topic subscriber for reading raw controller input	
	rospy.Subscriber("joy", Joy, callback)

	rospy.spin()

if __name__ == '__main__':
	try:
		main()

	except rospy.ROSInterruptException:
		pass