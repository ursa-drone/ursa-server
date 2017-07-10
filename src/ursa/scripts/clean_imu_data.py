#!/usr/bin/env python
import rospy
import math
from sensor_msgs.msg import Imu
import tf
import tf2_ros
import tf2_geometry_msgs
import geometry_msgs.msg

lastPub = 0
pub = rospy.Publisher('filtered_imu', Imu, queue_size=10)
tf_buffer = tf2_ros.Buffer(rospy.Duration(10.0)) #tf buffer length
tf_listener = tf2_ros.TransformListener(tf_buffer)

def callbackRaw(imu_in):
	global tf_buffer, lastPub
	try:
		transformToMap = tf_buffer.lookup_transform("base_link",
	               "map", #source frame
	               rospy.Time(0), #get the tf at first available time
	               rospy.Duration(0.5)) #wait for 0.5 seconds max
		# fakePose = geometry_msgs.msg.PoseStamped()
		# fakePose.pose.orientation = imu_in.orientation
		# newPose = tf2_geometry_msgs.do_transform_pose(fakePose, transform)
		# imu_in.orientation = newPose.pose.orientation
		br = tf2_ros.TransformBroadcaster()
		t = geometry_msgs.msg.TransformStamped()
		t.header.stamp = rospy.Time.now()
		t.header.frame_id = "base_link"
		t.child_frame_id = "imu_link"
		t.transform.translation.x = 0.0
		t.transform.translation.y = 0.0
		t.transform.translation.z = 0.0
		t.transform.rotation = transformToMap.transform.rotation
		br.sendTransform(t)
	except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):# Can't find the transform. This is expected from time to time.
		pass
	imu_in.orientation_covariance=(-1, 0, 0, 0, 0, 0, 0, 0, 0)
	if (lastPub == 0 or imu_in.header.stamp > lastPub):
		lastPub = imu_in.header.stamp
		pub.publish(imu_in)

    
def filter_imu():
	rospy.init_node('clean_imu_data', anonymous=True)
	subRaw = rospy.Subscriber('/mavros/imu/data_raw', Imu, callbackRaw)
	rospy.spin()

if __name__ == '__main__':
	filter_imu()

