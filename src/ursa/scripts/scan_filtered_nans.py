#!/usr/bin/env python
import rospy
import math
from sensor_msgs.msg import LaserScan

MAX_RANGE = 5.6
pub = rospy.Publisher('scan_filtered_nans', LaserScan, queue_size=10)

def callback(data):
    x = data.ranges
    data.ranges = ([MAX_RANGE + 0.1 if math.isnan(y) else y for y in x])
    data.range_max = MAX_RANGE + 1
    pub.publish(data)

def scan_filtered_nans():
    rospy.init_node('scan_filtered_nans', anonymous=True)
    sub = rospy.Subscriber('scan', LaserScan, callback)
    rospy.spin()

if __name__ == '__main__':
    scan_filtered_nans()

