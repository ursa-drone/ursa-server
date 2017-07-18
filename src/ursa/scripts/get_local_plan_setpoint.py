#!/usr/bin/env python
import rospy
import geometry_msgs.msg
import tf2_ros
import tf
import nav_msgs.msg
import geometry_msgs.msg

def callback(data):
    print '!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!', data.poses[-1]

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
    # r = rospy.Rate(1) # 50hz

    # setup publishers, subscribers and services
    rospy.init_node("get_local_plan_setpoint", anonymous=True)
    rospy.Subscriber('/move_base/URSAIPlannerROS/local_plan', nav_msgs.msg.Path, callback)


    # br= tf2_ros.TransformBroadcaster()

    # setpoint = geometry_msgs.msg.TransformStamped()
    # setpoint.header.stamp = rospy.Time.now()
    # setpoint.header.frame_id = "map"
    # setpoint.child_frame_id = "setpoint"
    # setpoint.transform.translation.x = 0.0
    # setpoint.transform.translation.y = 0.0
    # setpoint.transform.translation.z = 1.0
    # q = tf.transformations.quaternion_from_euler(0, 0, 0)
    # setpoint.transform.rotation.x = q[0]
    # setpoint.transform.rotation.y = q[1]
    # setpoint.transform.rotation.z = q[2]
    # setpoint.transform.rotation.w = q[3]

    # # send a few setpoints before starting
    # for i in range(100):
    #     br.sendTransform(setpoint)
    #     r.sleep()

    # # set mode
    # set_mode(0,"OFFBOARD")
    # arm(True)

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
    #     vel.publish(robot_vel)
    #     # br.sendTransform(setpoint)
    #     r.sleep()

    rospy.spin()

if __name__ == '__main__':
    print "getting local plan setpoint..."
    get_local_plan_setpoint()