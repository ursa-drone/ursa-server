/*
 * prefer_forward_cost_function.cpp
 *
 *  Created on: Apr 4, 2012
 *      Author: tkruse
 */

#include <ursa_local_planner/ursa_goal_front_cost.h>
#include <math.h>
#include <tf/transform_datatypes.h>

// debugging purposes
#include <iostream>
#include <ros/ros.h>
using namespace std;

namespace ursa_local_planner {


void UrsaPreferForwardCostFunction::init(double penalty, std::vector<geometry_msgs::PoseStamped> global_plan){
    penalty_ = penalty;
    global_plan_ = global_plan;
}


double UrsaPreferForwardCostFunction::scoreTrajectory(base_local_planner::Trajectory &traj) {
    int max_cost = global_plan_.size();
    double pose_yaw = tf::getYaw(global_plan_.front().pose.orientation);
    double goal_yaw = tf::getYaw(global_plan_.back().pose.orientation);
    // ROS_INFO("max_cost %d", max_cost);
    // ROS_INFO("possseee %f", max_cost*fabs(tf::getYaw(global_plan_.front().pose.orientation)/M_PI));
    // ROS_INFO("possseee %f %f %f %f", tf::getYaw(global_plan_.front().pose.orientation));
    // ROS_INFO("possseee %f", global_plan_.front().pose.orientation);
    // cout << "poseeee: " << global_plan_.front().pose.orientation << endl;

    // ROS_INFO("diff %f", max_cost*fabs(pose_yaw-goal_yaw)/M_PI);
    return 2*max_cost*fabs(pose_yaw-goal_yaw)/M_PI;
}

}