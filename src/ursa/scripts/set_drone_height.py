#!/usr/bin/env python
import rospy
import geometry_msgs.msg
import tf2_ros
import tf
import mavros_msgs.srv
import mavros_msgs.msg
import sensor_msgs.msg

FCU_connected = False

def get_current_state(data):
    global FCU_connected
    FCU_connected = data.connected

def set_drone_height_pose():
    r = rospy.Rate(10) # 10hz

    # setup publishers, subscribers and services
    state = rospy.Subscriber('/mavros/state', mavros_msgs.msg.State, get_current_state)
    local_pos_pub = rospy.Publisher('/mavros/setpoint_position/local', geometry_msgs.msg.PoseStamped, queue_size=10)
    set_mode = rospy.ServiceProxy('/mavros/set_mode', mavros_msgs.srv.SetMode)
    arm = rospy.ServiceProxy('/mavros/cmd/arming', mavros_msgs.srv.CommandBool)

    # wait for FCU connection
    while ( (not FCU_connected) & (not rospy.is_shutdown()) ):
        r.sleep()
    print "FCU connected...."

    # set pose
    pose = geometry_msgs.msg.PoseStamped()
    pose.header.stamp = rospy.Time.now()
    pose.header.frame_id = "drone_link"
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 1;
    q = tf.transformations.quaternion_from_euler(0, 0, 0)
    pose.pose.orientation.x = q[0]
    pose.pose.orientation.y = q[1]
    pose.pose.orientation.z = q[2]
    pose.pose.orientation.w = q[3]

    # send a few setpoints before starting
    for i in range(100):
        local_pos_pub.publish(pose)
        r.sleep()

    # set mode
    set_mode(0,"OFFBOARD" )
    arm(True)

    # publish pose
    while not rospy.is_shutdown():
        local_pos_pub.publish(pose)
        r.sleep()

def set_drone_height_setpoint():
    r = rospy.Rate(10) # 10hz

    # setup publishers, subscribers and services
    state = rospy.Subscriber('/mavros/state', mavros_msgs.msg.State, get_current_state)
    set_mode = rospy.ServiceProxy('/mavros/set_mode', mavros_msgs.srv.SetMode)
    arm = rospy.ServiceProxy('/mavros/cmd/arming', mavros_msgs.srv.CommandBool)
    vel = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel_unstamped', geometry_msgs.msg.Twist, queue_size=10)

    # wait for FCU connection
    while ( (not FCU_connected) & (not rospy.is_shutdown()) ):
        r.sleep()
    print "FCU connected...."

    br= tf2_ros.TransformBroadcaster()

    setpoint = geometry_msgs.msg.TransformStamped()
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

    # send a few setpoints before starting
    for i in range(100):
        br.sendTransform(setpoint)
        r.sleep()

    # set mode
    set_mode(0,"OFFBOARD")
    arm(True)

    # robot_vel = geometry_msgs.msg.Twist()
    # # robot_vel.header.frame_id = "map"
    # robot_vel.linear.x = 0
    # robot_vel.linear.y = 0
    # robot_vel.linear.z = 2
    # robot_vel.angular.x = 0
    # robot_vel.angular.y = 0
    # robot_vel.angular.z = 2

    # while not rospy.is_shutdown():
    #     # robot_vel.header.stamp = rospy.Time.now()
    #     # vel.publish(robot_vel)
    #     br.sendTransform(setpoint)
    #     r.sleep()

    rospy.spin()

if __name__ == '__main__':
    rospy.init_node("set_drone_height", anonymous=True)
    print "setting drone height...."
    set_drone_height_setpoint()