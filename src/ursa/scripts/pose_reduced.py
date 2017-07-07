#!/usr/bin/env python
import rospy
import tf2_ros
import geometry_msgs.msg as g

rospy.init_node('pose', anonymous=True)

# set_mode(0,"OFFBOARD" )
# arm(True)

def sendTransform(data):
    # initialise transform msg
    t = g.TransformStamped()

    # Populate transform data
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = 'map'
    t.child_frame_id = 'laser'

    # set transform translation values
    t.transform.translation.x = data.x
    t.transform.translation.y = data.y
    t.transform.translation.z = 0

    # set transform rotation values
    q = tf.transformations.quaternion_from_euler(0, 0, 0)
    t.transform.rotation.x = q[0]
    t.transform.rotation.y = q[1]
    t.transform.rotation.z = q[2]
    t.transform.rotation.w = q[3]
    # rospy.logout(data)

    # Send transform data
    tfB = tf2_ros.TransformBroadcaster()
    tfB.sendTransform(t)



# pose = g.PoseStamped()


# setpoint = geometry_msgs.msg.TransformStamped()
# tf_buffer = tf2_ros.Buffer(rospy.Duration(10.0)) #tf buffer length
# tf_listener = tf2_ros.TransformListener(tf_buffer)

# rate = rospy.Rate(20) # 20hz

# local_pos_pub = rospy.Publisher('/mavros/setpoint_position/local', geometry_msgs.msg.PoseStamped, queue_size=10)
# pose.pose.position.x = 0
# pose.pose.position.y = 0
# pose.pose.position.z = 1
# local_pos_pub.publish(pose)

# set_mode = rospy.ServiceProxy('/mavros/set_mode', mavros_msgs.srv.SetMode)  
# arm = rospy.ServiceProxy('/mavros/cmd/arming', mavros_msgs.srv.CommandBool)  

# br= tf2_ros.TransformBroadcaster()
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

# for i in range(100):
#     #local_pos_pub.publish(pose)
#     br.sendTransform(setpoint)
#     rate.sleep()

# set_mode(0,"OFFBOARD" )
# arm(True)

# while not rospy.is_shutdown():
#     #local_pos_pub.publish(pose)
#     #br.sendTransform(setpoint)
#     rate.sleep()