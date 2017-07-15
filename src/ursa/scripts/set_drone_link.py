#!/usr/bin/env python
import rospy
import sensor_msgs.msg
import tf2_ros
import tf
import geometry_msgs.msg

def set_drone_link(data):
    br = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()

    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "base_link"
    t.child_frame_id = "drone_link"
    t.transform.translation.x = 0
    t.transform.translation.y = 0
    t.transform.translation.z = data.range
    q = tf.transformations.quaternion_from_euler(0, 0, 0)
    t.transform.rotation.x = q[0]
    t.transform.rotation.y = q[1]
    t.transform.rotation.z = q[2]
    t.transform.rotation.w = q[3]

    br.sendTransform(t)

if __name__ == '__main__':
    print "setting drone link...."
    rospy.init_node("set_drone_link", anonymous=True)
    sub = rospy.Subscriber("range", sensor_msgs.msg.Range, set_drone_link, queue_size=10)
    rospy.spin()