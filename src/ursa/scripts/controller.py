#!/usr/bin/env python
import rospy
import geometry_msgs.msg
import mavros_msgs.srv
import mavros_msgs.msg
import tf2_ros

def set_position(x, y, z):
    br = tf2_ros.TransformBroadcaster()
    setpoint = geometry_msgs.msg.TransformStamped()
    setpoint.header.stamp = rospy.Time.now()
    setpoint.header.frame_id = "map"
    setpoint.child_frame_id = "setpoint"
    setpoint.transform.translation.x = x
    setpoint.transform.translation.y = y
    setpoint.transform.translation.z = z
    setpoint.transform.rotation.w = 1
    br.sendTransform(setpoint)

if __name__ == '__main__':
    rospy.init_node('ursa_controller', anonymous=True)
    set_mode = rospy.ServiceProxy('/mavros/set_mode', mavros_msgs.srv.SetMode)  
    arm = rospy.ServiceProxy('/mavros/cmd/arming', mavros_msgs.srv.CommandBool)  
    rate=rospy.Rate(20)
    for i in range(20*15):
        set_position(0, 0, 1)
        rate.sleep()
    set_mode(0,"OFFBOARD" )
    arm(True)
    for i in range(20*10):
        rate.sleep()
    set_position(0, 0, 0)
    for i in range(20*10):
        rate.sleep()
    arm(False)