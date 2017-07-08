#!/usr/bin/env python
import rospy
import geometry_msgs.msg
import sensor_msgs.msg
import tf
import tf2_ros
import tf2_geometry_msgs
import mavros_msgs.srv
import mavros_msgs.msg
import nav_msgs.msg
import math

def rangeCB(data):
    br = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "base_link"
    t.child_frame_id = "drone_link"
    t.transform.translation.x = 0.0
    t.transform.translation.y = 0.0
    t.transform.translation.z = data.range
    q = tf.transformations.quaternion_from_euler(0, 0, 0)
    t.transform.rotation.x = q[0]
    t.transform.rotation.y = q[1]
    t.transform.rotation.z = q[2]
    t.transform.rotation.w = q[3]
    br.sendTransform(t)

 
if __name__ == '__main__':
    rospy.init_node('height_publisher', anonymous=True)
    print("Initialising height_publisher node...")
    range_sub = rospy.Subscriber('/mavros/distance_sensor/sonar_sensor', sensor_msgs.msg.Range, rangeCB, queue_size=10)
    rospy.spin()