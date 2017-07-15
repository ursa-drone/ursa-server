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

def setpoint_buffer(rate):
    for i in range(10):
        print i,
        set_position(0, 0, 1)
        rate.sleep()

if __name__ == '__main__':
    rospy.init_node('ursa_controller', anonymous=True)
    rate = rospy.Rate(20)

    # setup services
    set_mode = rospy.ServiceProxy('/mavros/set_mode', mavros_msgs.srv.SetMode)
    arm = rospy.ServiceProxy('/mavros/cmd/arming', mavros_msgs.srv.CommandBool)
    # rc_override = rospy.Publisher('/mavros/rc/override', mavros_msgs.msg.OverrideRCIn)

    # set_mode(0, "POSCTL")
    # arm(True)

    # for i in range(8):
    #     rc_cmds = mavros_msgs.msg.OverrideRCIn()
    #     for j in range(100):
    #         rc_cmds.channels[i] = 1
    #         rc_override.publish(rc_cmds)
    #         rate.sleep()

    # rospy.spin()

    previous_input = None
    while not rospy.is_shutdown():
        # get user input
        user_input = raw_input()
        print ">>> ", user_input

        if user_input == 'l':
            set_mode(0, "AUTO.LAND")
        elif user_input == 't':
            setpoint_buffer(rate)
            set_mode(0, "OFFBOARD")
            arm(True)
        elif user_input == 'd':
            arm(False)
        elif user_input == 'p':
            set_mode(0, "POSCTL")

        rate.sleep()