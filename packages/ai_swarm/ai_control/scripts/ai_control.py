#!/usr/bin/env python
import rospy
from std_msgs.msg import Int8, Int16
import time

# 1000 Forward
# 0100 Reverse
# 0010 Right
# 0001 Left


pub = rospy.Publisher('ez_main_topic', Int16, queue_size=100)
rospy.init_node('ai_control_node', anonymous=True)

rate = rospy.Rate(600) # 600hz

commands = {'forward' : 0b100000000000, 'reverse' : 0b010000000000, 'left' : 0b001000000000, 'right' : 0b000100000000, 
                'front_arm_up' : 0b000010000000, 'front_arm_down' : 0b000001000000, 'back_arm_up' : 0b000000100000, 'back_arm_down' : 0b000000010000,
                'front_dig' : 0b000000001000, 'front_dump' : 0b000000000100, 'back_dig' : 0b0000000000100, 'back_dump' : 0b000000000001}
                


def raise_arms():
    start = time.time()
    while time.time() - start < 1.9:
        pub.publish(commands['front_arm_up'] | commands['back_arm_up'])

def ai_control():
    global pub
    rotate_time = 2
    drive_time = 15

    raise_arms()

    while not rospy.is_shutdown():

        start_time = time.time()

        if (time.time() - start_time - drive_time) < 0:
            #print(temp)
            pub.publish(commands['forward'])
        elif (time.time() - start_time - drive_time + rotate_time) < 0:
            #print(temp)
            pub.publish(commands['front_arm_down'] | commands['back_arm_down'])
        elif (time.time() - start_time - 2*drive_time + rotate_time) < 0:
            #print(temp)
            pub.publish(commands['reverse'])
        
                
        rate.sleep()


if __name__ == "__main__":
    try:
        ai_control()
    except rospy.ROSInterruptException:
        pass