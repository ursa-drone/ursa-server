#!/usr/bin/env python
import rospy
import geometry_msgs.msg
import sensor_msgs.msg
import move_base_msgs.msg
import tf
import tf2_ros
import tf2_geometry_msgs
import mavros_msgs.srv
import mavros_msgs.msg
import nav_msgs.msg
import curses
import math

pose = geometry_msgs.msg.PoseStamped()
setpoint = geometry_msgs.msg.TransformStamped()
tf_buffer = tf2_ros.Buffer(rospy.Duration(10.0)) #tf buffer length
tf_listener = tf2_ros.TransformListener(tf_buffer)

def start():
    rate = rospy.Rate(20) # 20hz

    local_pos_pub = rospy.Publisher('/mavros/setpoint_position/local', geometry_msgs.msg.PoseStamped, queue_size=10)
    pose.pose.position.x = 0
    pose.pose.position.y = 0
    pose.pose.position.z = 1
    local_pos_pub.publish(pose)

    set_mode = rospy.ServiceProxy('/mavros/set_mode', mavros_msgs.srv.SetMode)  
    arm = rospy.ServiceProxy('/mavros/cmd/arming', mavros_msgs.srv.CommandBool)  

    br= tf2_ros.TransformBroadcaster()
    setpoint.header.stamp = rospy.Time.now()
    setpoint.header.frame_id = "map"
    setpoint.child_frame_id = "setpoint"
    setpoint.transform.translation.x = 0.0
    setpoint.transform.translation.y = 0.0
    setpoint.transform.translation.z = 1.0
    q = tf.transformations.quaternion_from_euler(0, 0, 0)
    setpoint.transform.rotation.x = q[0]
    setpoint.transform.rotation.y = q[1]
    setpoint.transform.rotation.z = q[2]
    setpoint.transform.rotation.w = q[3]

    for i in range(100):
        #local_pos_pub.publish(pose)
        br.sendTransform(setpoint)
        rate.sleep()

    set_mode(0,"OFFBOARD" )
    arm(True)

    while not rospy.is_shutdown():
        #local_pos_pub.publish(pose)
        #br.sendTransform(setpoint)
        rate.sleep()


 
if __name__ == '__main__':
    rospy.init_node('pose', anonymous=True)
    start()