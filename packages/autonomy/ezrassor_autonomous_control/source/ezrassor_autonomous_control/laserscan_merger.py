#!/usr/bin/env python

import rospy
import numpy as np
from message_filters import ApproximateTimeSynchronizer, Subscriber
from sensor_msgs.msg import LaserScan
from copy import deepcopy

pub = rospy.Publisher('obstacle_detection/combined', LaserScan, queue_size=10)

def merge_laserscans(ls1, ls2):
    # start with copy of ls1 for filling out header
    scan = deepcopy(ls1)
    # update time stamp in header
    scan.header.stamp = rospy.Time.now()
    # set ranges as the min (excluding NaNs where possible) of the two given LaserScans
    scan.ranges = [np.nanmin((x, y)) for (x, y) in zip(ls1.ranges, ls2.ranges)]
    pub.publish(scan)

def laserscan_merger(slop=0.5):
    rospy.init_node('laserscan_merger')
    rospy.loginfo('Laserscan Merger initialized.')
    sub1 = Subscriber('obstacle_detection/farthest_point', LaserScan)
    sub2 = Subscriber('obstacle_detection/floor_proj', LaserScan)
    tss = ApproximateTimeSynchronizer([sub1, sub2], queue_size=5, slop=slop)
    tss.registerCallback(merge_laserscans)
    rospy.spin()

if __name__ == "__main__":
    try:
        laserscan_merger()
    except:
        pass
