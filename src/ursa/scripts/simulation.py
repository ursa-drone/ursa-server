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
import nav_msgs.msg

pose = geometry_msgs.msg.PoseStamped()
setpoint = geometry_msgs.msg.TransformStamped()
tf_buffer = tf2_ros.Buffer(rospy.Duration(10.0)) #tf buffer length
tf_listener = tf2_ros.TransformListener(tf_buffer)
robot_radius = rospy.get_param("move_base/global_costmap/robot_radius")
global_plan_endpoint = geometry_msgs.msg.PoseStamped()
current_pose = geometry_msgs.msg.PoseStamped()


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

def odometryCB(data):
    global current_pose
    current_pose.header = data.header
    current_pose.pose = data.pose.pose

def setGlobalPlanCB(data):
    global global_plan_endpoint
    global_plan_endpoint = data
    # print "global_plan_endpoint: ", global_plan_endpoint

def waypointCB(data):
    global tf_buffer
    rate = rospy.Rate(20) # 20hz
    transform = tf_buffer.lookup_transform("map",
                                       data.header.frame_id, #source frame
                                       rospy.Time(0), #get the tf at first available time
                                       rospy.Duration(1.0)) #wait for 1 second

    # If goal is inside robot foot print then setpoint as goal opposed to local plan
    # Commented out in response to issue #2.  Set local planner orientation in trajectory generator when inside robot radius.
    # if (((current_pose.pose.position.x - robot_radius) < global_plan_endpoint.pose.position.x) and
    #     (global_plan_endpoint.pose.position.x < (current_pose.pose.position.x + robot_radius)) and
    #     ((current_pose.pose.position.y - robot_radius) < global_plan_endpoint.pose.position.y) and
    #     (global_plan_endpoint.pose.position.y < (current_pose.pose.position.y + robot_radius))):
    #     mapPose = global_plan_endpoint
    # else:
    #     mapPose = tf2_geometry_msgs.do_transform_pose(data.poses[-1], transform)

    mapPose = tf2_geometry_msgs.do_transform_pose(data.poses[-1], transform)

    global setpoint
    br=tf2_ros.TransformBroadcaster()
    setpoint.header.frame_id = "map"
    setpoint.child_frame_id = "setpoint"
    setpoint.transform.translation.x = mapPose.pose.position.x
    setpoint.transform.translation.y = mapPose.pose.position.y
    setpoint.transform.translation.z = 1.0
    setpoint.transform.rotation.x = mapPose.pose.orientation.x
    setpoint.transform.rotation.y = mapPose.pose.orientation.y
    setpoint.transform.rotation.z = mapPose.pose.orientation.z
    setpoint.transform.rotation.w = mapPose.pose.orientation.w
    br.sendTransform(setpoint)
    rate.sleep()

def start():
    global pose
    global setpoint
    rate = rospy.Rate(20) # 20hz
    rate.sleep()
    
    local_pos_pub = rospy.Publisher('/mavros/setpoint_position/local', geometry_msgs.msg.PoseStamped, queue_size=10)
    pose.pose.position.x = 0
    pose.pose.position.y = 0
    pose.pose.position.z = 1
    local_pos_pub.publish(pose)

    waypoints=[mavros_msgs.msg.Waypoint()]
    waypoints[0].frame=0
    waypoints[0].command=16
    waypoints[0].is_current=1
    waypoints[0].autocontinue=0
    waypoints[0].param1=100
    waypoints[0].param2=0.1
    waypoints[0].param3=0
    waypoints[0].param4=0
    waypoints[0].x_lat=0
    waypoints[0].y_long=0
    waypoints[0].z_alt=2

    odom_sub = rospy.Subscriber('mavros/local_position/odom',nav_msgs.msg.Odometry,odometryCB)
    local_plan_sub = rospy.Subscriber('/move_base/UrsaPlannerROS/local_plan', nav_msgs.msg.Path, waypointCB, queue_size=10)
    simple_goal_sub = rospy.Subscriber('move_base_simple/goal', geometry_msgs.msg.PoseStamped, setGlobalPlanCB, queue_size=10)
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


def takeoff():
    takeoff = rospy.ServiceProxy('/mavros/cmd/takeoff', mavros_msgs.srv.CommandTOL)  
    print "takeoff: ", takeoff(0,0,0,0,40)
    

def land():
    set_mode = rospy.ServiceProxy('/mavros/set_mode', mavros_msgs.srv.SetMode)  
    print "set mode: ", set_mode(208,'GUIDED')

    land = rospy.ServiceProxy('/mavros/cmd/land', mavros_msgs.srv.CommandTOL)  
    print "land: ", land(0,0,0,0,0)

 
if __name__ == '__main__':
    rospy.init_node('pose', anonymous=True)
    start()