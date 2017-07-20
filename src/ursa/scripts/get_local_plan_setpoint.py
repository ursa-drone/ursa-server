#!/usr/bin/env python
import rospy
import geometry_msgs.msg
import tf2_ros
import tf
import nav_msgs.msg
import geometry_msgs.msg

def callback(data):

    nav_pose = data.poses[-1]

    br= tf2_ros.TransformBroadcaster()

    setpoint = geometry_msgs.msg.TransformStamped()
    setpoint.header.stamp = rospy.Time.now()
    setpoint.header.frame_id = "map"
    setpoint.child_frame_id = "setpoint"
    setpoint.transform.translation.x = nav_pose.pose.position.x
    setpoint.transform.translation.y = nav_pose.pose.position.y
    setpoint.transform.translation.z = 1
    setpoint.transform.rotation.x = nav_pose.pose.orientation.x
    setpoint.transform.rotation.y = nav_pose.pose.orientation.y
    setpoint.transform.rotation.z = nav_pose.pose.orientation.z
    setpoint.transform.rotation.w = nav_pose.pose.orientation.w

    br.sendTransform(setpoint)

def get_local_plan_setpoint():
    rospy.init_node("get_local_plan_setpoint", anonymous=True)
    rospy.Subscriber('/move_base/DWAPlannerROS/local_plan', nav_msgs.msg.Path, callback)

    rospy.spin()

if __name__ == '__main__':
    print "getting local plan setpoint..."
    get_local_plan_setpoint()