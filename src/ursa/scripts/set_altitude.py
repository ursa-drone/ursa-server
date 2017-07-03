#!/usr/bin/env python
import rospy
import geometry_msgs.msg as g
import mavros_msgs.srv
import mavros_msgs.msg

def set_position(x, y, z):
    ps = g.PoseStamped()

    set_mode = rospy.ServiceProxy('/mavros/set_mode', mavros_msgs.srv.SetMode)
    arm = rospy.ServiceProxy('/mavros/cmd/arming', mavros_msgs.srv.CommandBool)

    # set_mode(0, "OFFBOARD")
    # arm(True)

    ps.pose.position.x = x
    ps.pose.position.y = y
    ps.pose.position.z = z

    altitude = rospy.Publisher('/mavros/setpoint_position/local', g.PoseStamped, queue_size=10)
    rospy.init_node('set_position', anonymous=True)

    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        rospy.loginfo(ps)
        altitude.publish(ps)
        rate.sleep()

if __name__ == '__main__':
    try:
        set_position(0, 0, 1)
    except rospy.ROSInterruptException:
        pass