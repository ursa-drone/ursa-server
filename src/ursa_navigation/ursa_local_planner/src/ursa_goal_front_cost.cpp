/*
 * prefer_forward_cost_function.cpp
 *
 *  Created on: Apr 4, 2012
 *      Author: tkruse
 */

#include <ursa_local_planner/ursa_goal_front_cost.h>
#include <math.h>

// debugging purposes
#include <iostream>
#include <ros/ros.h>

const double TOLERANCE = M_PI * 90 / 180;

using namespace std;

namespace ursa_local_planner {


void UrsaPreferForwardCostFunction::init(double penalty, tf::Stamped<tf::Pose> global_pose){
    // ROS_INFO("########## init ############");
    penalty_ = penalty;
    global_pose_ = global_pose;
}


double UrsaPreferForwardCostFunction::scoreTrajectory(base_local_planner::Trajectory &traj) {
    double x, y, th;
    Eigen::Vector3f pos(global_pose_.getOrigin().getX(), global_pose_.getOrigin().getY(), tf::getYaw(global_pose_.getRotation()));
    // ROS_INFO("traj_endpoint: x=.%f y=.%f th=.%f", x, y, th);

    double pose_yaw = pos[2];

    traj.getPoint(0,x, y, th);
    double goal_yaw_begin = th;
    traj.getEndpoint(x, y, th);
    double goal_yaw_end = th;

    // ROS_INFO("TOLERANCE - %f", TOLERANCE);
    // ROS_INFO("max_cost %d", max_cost);
    // ROS_INFO("possseee %f", max_cost*fabs(tf::getYaw(global_plan_.front().pose.orientation)/M_PI));
    // ROS_INFO("possseee %f %f %f %f", tf::getYaw(global_plan_.front().pose.orientation));
    // ROS_INFO("possseee %f", global_plan_.front().pose.orientation);
    // cout << "poseeee: " << global_plan_.front().pose.orientation << endl;

    // ROS_INFO("diff %f", max_cost*fabs(pose_yaw-goal_yaw)/M_PI);
    // return 1.0;
    if (fabs(pose_yaw-goal_yaw_begin) > TOLERANCE){
        // ROS_INFO("return -1. pose_yaw: %0.2f, goal_yaw_b: %0.2f, goal_yaw_e: %0.2f", pose_yaw, goal_yaw_begin, goal_yaw_end);
        return 1.0;
    }else{
        // ROS_INFO("return 1. pose_yaw: %0.2f, goal_yaw_b: %0.2f, goal_yaw_e: %0.2f", pose_yaw, goal_yaw_begin, goal_yaw_end);
        return 1.0;
    }
}

}