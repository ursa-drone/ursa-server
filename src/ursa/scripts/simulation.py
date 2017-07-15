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
import PyKDL

pose = geometry_msgs.msg.PoseStamped()
setpoint = geometry_msgs.msg.TransformStamped()
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

# def waypointCB(data):
#     global tf_buffer
#     transform = tf_buffer.lookup_transform("map",
#                                        data.header.frame_id, #source frame
#                                        rospy.Time(0), #get the tf at first available time
#                                        rospy.Duration(1.0)) #wait for 1 second
#     mapPose = tf2_geometry_msgs.do_transform_pose(data.poses[-1], transform)
#     global setpoint
#     br=tf2_ros.TransformBroadcaster()
#     setpoint.header.frame_id = "map"
#     setpoint.child_frame_id = "setpoint"
#     setpoint.transform.translation.x = mapPose.pose.position.x
#     setpoint.transform.translation.y = mapPose.pose.position.y
#     setpoint.transform.translation.z = 1.0
#     setpoint.transform.rotation.x = mapPose.pose.orientation.x
#     setpoint.transform.rotation.y = mapPose.pose.orientation.y
#     setpoint.transform.rotation.z = mapPose.pose.orientation.z
#     setpoint.transform.rotation.w = mapPose.pose.orientation.w
#     br.sendTransform(setpoint)
#     rate = rospy.Rate(20) # 20hz
#     rate.sleep()

def start():
    global pose
    global setpoint
    rate = rospy.Rate(20) # 20hz
    rate.sleep()

    # goal_sub = rospy.Subscriber('/move_base/UrsaPlannerROS/local_plan', nav_msgs.msg.Path, waypointCB, queue_size=10)
    range_sub = rospy.Subscriber('/range', sensor_msgs.msg.Range, rangeCB, queue_size=10)
    set_mode = rospy.ServiceProxy('/mavros/set_mode', mavros_msgs.srv.SetMode)  
    arm = rospy.ServiceProxy('/mavros/cmd/arming', mavros_msgs.srv.CommandBool)  
    #setWaypoints = rospy.ServiceProxy('/mavros/mission/push', mavros_msgs.srv.WaypointPush) 

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