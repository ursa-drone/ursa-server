
/*
 * prefer_forward_cost_function.cpp
 *
 *  Created on: Apr 4, 2012
 *      Author: tkruse
 */

#include <ursa_local_planner/ursa_sticky_cost.h>
//DEBUG
#include <ros/ros.h>
#include <iostream>
using namespace std;

#include <math.h>
#include <algorithm> // std::min_element

namespace ursa_local_planner {


bool UrsaStickyCostFunction::init(double penalty, 
                                std::vector<geometry_msgs::PoseStamped> global_plan, 
                                tf::Stamped<tf::Pose> global_pose, 
                                double robot_radius,
                                base_local_planner::Trajectory previous_result_traj){
    penalty_=penalty;
    global_plan_=global_plan;
    global_pose_=global_pose;
    robot_radius_=robot_radius;
    previous_traj_=previous_result_traj;
}

inline double euclidean_distance(double x1, double x2, double y1, double y2)
{
    double x_diff = x1 - x2;
    double y_diff = y1 - y2;
    double distance_sq = x_diff*x_diff + y_diff*y_diff;
    return sqrt(distance_sq);
}

double UrsaStickyCostFunction::scoreTrajectory(base_local_planner::Trajectory &traj) {
    // trajectory end point - previous
    double x_prev, y_prev, th_prev;
    if (previous_traj_.getPointsSize()>0){
        previous_traj_.getEndpoint(x_prev, y_prev, th_prev);
    } else {
        return 0;
    }


    // trajectory end point
    double x_end, y_end, th_end;
    traj.getEndpoint(x_end, y_end, th_end);

    if (euclidean_distance(x_prev,x_end,y_prev,y_end)<0.01){
        //ROS_INFO("Found same traj - score zero");
        return 0;
    }
    // current pose
    double current_pose_x = global_pose_ .getOrigin().getX();
    double current_pose_y = global_pose_.getOrigin().getY();
    double current_pose_th = tf::getYaw(global_pose_.getRotation());

    double distance_to_prev = euclidean_distance(x_prev,current_pose_x,y_prev,current_pose_y);

    // cost
    //return -1;
    //return 0;
    return 220*exp(-1.0f/distance_to_prev);

    // // for each point in global plan, check distance from traj to point
    // int i = 0;
    // std::vector<double> dist_traj_to_Sticky;
    // std::vector<geometry_msgs::PoseStamped>::iterator poseIt;
    // for (poseIt=global_plan_.begin(); poseIt < global_plan_.end(); poseIt++){
    //     geometry_msgs::PoseStamped& w = *poseIt;
    //     dist_traj_to_Sticky.push_back(euclidean_distance(x_end, w.pose.position.x, y_end, w.pose.position.y));
    // }

    // // get index of global plan point where distance is minimum to trajectory
    // double min_index = min_element(dist_traj_to_Sticky.begin(), dist_traj_to_Sticky.end()) - dist_traj_to_Sticky.begin();

    // cost = 100 * (1 - min_index/global_plan_.size());
    // //ROS_INFO("Sticky cost -- 1 -- %f", cost); // returns cost of last point on trajectory
    // return cost;
    // }
}

} // NAMESPACE