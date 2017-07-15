#!/usr/bin/env python
import rospy
import math
from sensor_msgs.msg import Imu
import tf
import tf2_ros
import tf2_geometry_msgs
import geometry_msgs.msg

lastPub = 0
lastClean = 0
pub = rospy.Publisher('filtered_imu', Imu, queue_size=10)
tf_buffer = tf2_ros.Buffer(rospy.Duration(10.0)) #tf buffer length
tf_listener = tf2_ros.TransformListener(tf_buffer)

def callbackRaw(imu_in):
	global lastPub, lastClean
	if (lastClean != 0 and lastClean > rospy.Time.now() - rospy.Duration(1)):
		return #Don't publish raw data if clean data is available from estimator
	if (lastPub == 0 or imu_in.header.stamp > lastPub):
		lastPub = imu_in.header.stamp
		pub.publish(imu_in)

def callbackData(imu_in):
	global lastPub, lastClean
	if (lastPub == 0 or imu_in.header.stamp > lastPub):
		lastPub = imu_in.header.stamp
		lastClean = imu_in.header.stamp
		pub.publish(imu_in)

def filter_imu():
	rospy.init_node('clean_imu_data', anonymous=True)
	subRaw = rospy.Subscriber('/mavros/imu/data_raw', Imu, callbackRaw)
	subData = rospy.Subscriber('/mavros/imu/data', Imu, callbackData)
	rospy.spin()

if __name__ == '__main__':
	filter_imu()

