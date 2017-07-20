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
import PyKDL

tf_buffer = tf2_ros.Buffer(rospy.Duration(10.0)) #tf buffer length
tf_listener = tf2_ros.TransformListener(tf_buffer)

def rangeCB(data):
    global tf_buffer
    t = geometry_msgs.msg.TransformStamped()
    br = tf2_ros.TransformBroadcaster()
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "base_link"
    t.child_frame_id = "drone_link"
    t.transform.rotation.w = 1.0
    try:
        transform_base = tf_buffer.lookup_transform("map",
               "base_link", #source frame
               rospy.Time(0), #get the tf at first available time
               rospy.Duration(1)) #wait for 1 second
        rot2Base = PyKDL.Rotation.Quaternion(transform_base.transform.rotation.x, 
                                                transform_base.transform.rotation.y, 
                                                transform_base.transform.rotation.z, 
                                                transform_base.transform.rotation.w)
        trans2Base = PyKDL.Vector(transform_base.transform.translation.x,transform_base.transform.translation.y,transform_base.transform.translation.z)
        referenceHeight = PyKDL.Vector(0,0,data.range)
        transformedHeight = rot2Base.Inverse()*referenceHeight
        mapPos = PyKDL.Vector(transform_base.transform.translation.x,transform_base.transform.translation.y,transformedHeight.z())
        transformedPos = PyKDL.Frame(rot2Base,trans2Base).Inverse()*mapPos
        t.transform.translation.x = transformedPos.x()
        t.transform.translation.y = transformedPos.y()
        t.transform.translation.z = transformedPos.z()
    except:
        t.transform.translation.x = 0
        t.transform.translation.y = 0
        t.transform.translation.z = data.range
    br.sendTransform(t)

 
if __name__ == '__main__':
    rospy.init_node('height_publisher', anonymous=True)
    print("Initialising height_publisher node...")
    range_sub = rospy.Subscriber('/mavros/distance_sensor/sonar_sensor', sensor_msgs.msg.Range, rangeCB, queue_size=10)
    rospy.spin()