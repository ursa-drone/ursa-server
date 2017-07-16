#!/usr/bin/env python
import rospy
import geometry_msgs.msg
import mavros_msgs.srv
import mavros_msgs.msg
import tf2_ros
from pynput import keyboard

# setup node
rospy.init_node('ursa_controller', anonymous=True)
rate = rospy.Rate(20)
set_mode = rospy.ServiceProxy('/mavros/set_mode', mavros_msgs.srv.SetMode)
arm = rospy.ServiceProxy('/mavros/cmd/arming', mavros_msgs.srv.CommandBool)
x = 0; y = 0; z = 0

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

def setpoint_buffer(x, y, z):
    for i in range(10):
        print i,
        set_position(x, y, z)
        rate.sleep()

def on_press(key):
    global x, y, z
    MOVEMENT_SENSITIVITY = 0.1

    try: k = key.char
    except: k = key.name

    # determine actions on key press
    if k == 'left':
        print 'left command received...'
        x -= MOVEMENT_SENSITIVITY
        set_position(x, y, z)
    elif k == 'right':
        print 'right command received...'
        x += MOVEMENT_SENSITIVITY
        set_position(x, y, z)
    elif k == 'down':
        print 'down command received...'
        y -= MOVEMENT_SENSITIVITY
        set_position(x, y, z)
    elif k == 'up':
        print 'up command received...'
        y += MOVEMENT_SENSITIVITY
        set_position(x, y, z)
    elif k == 't':
        print 'takeoff command received...'
        z = 1
        set_position(x, y, z)
        set_mode(0, "OFFBOARD")
        arm(True)
    elif k == 'l':
        print 'land command received...'
        set_mode(0, "AUTO.LAND")
        previous_input = 'l'
    elif k == 'd':
        print 'disarm command received...'
        arm(False)

    print "key pressed"
    rate.sleep()

if __name__ == '__main__':
    lis = keyboard.Listener(on_press=on_press)
    lis.start()

    rospy.spin()




