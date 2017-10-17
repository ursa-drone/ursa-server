
/*
 * prefer_forward_cost_function.cpp
 *
 *  Created on: Apr 4, 2012
 *      Author: tkruse
 */

#include <ursa_local_planner/ursa_goal_cost.h>
//DEBUG
#include <ros/ros.h>
#include <iostream>
using namespace std;

#include <math.h>
#include <algorithm> // std::min_element

namespace ursa_local_planner {


bool UrsaGoalCostFunction::init(double penalty, 
                                std::vector<geometry_msgs::PoseStamped> global_plan, 
                                tf::Stamped<tf::Pose> global_pose, 
                                double robot_radius,
                                base_local_planner::Trajectory previous_result_traj){
    penalty_=penalty;
    global_plan_=global_plan;
    global_pose_=global_pose;
    robot_radius_=robot_radius;
}

inline double euclidean_distance(double x1, double x2, double y1, double y2)
{
    double x_diff = x1 - x2;
    double y_diff = y1 - y2;
    double distance_sq = x_diff*x_diff + y_diff*y_diff;
    return sqrt(distance_sq);
}

double UrsaGoalCostFunction::scoreTrajectory(base_local_planner::Trajectory &traj) {
    // setup
    double cost;

    // trajectory end point
    double x_end, y_end, th_end;
    traj.getEndpoint(x_end, y_end, th_end);

    // current pose
    double current_pose_x = global_pose_ .getOrigin().getX();
    double current_pose_y = global_pose_.getOrigin().getY();
    double current_pose_th = tf::getYaw(global_pose_.getRotation());

    // goal co-ordinates
    double goal_x = global_plan_.back().pose.position.x;
    double goal_y = global_plan_.back().pose.position.y;
    double goal_th = tf::getYaw(global_plan_.back().pose.orientation);

    //goal distance
    double goal_dist = euclidean_distance(x_end,goal_x,y_end,goal_y);

    //home distance to goal
    double home_dist = euclidean_distance(x_end,current_pose_x,y_end,current_pose_y);

    // cost
    //return 100*exp(-30.0f*(home_dist)/(home_dist+goal_dist));

    unsigned int index=traj.getIndex();
    return 100*exp(-1.0f*index);

    // // for each point in global plan, check distance from traj to point
    // int i = 0;
    // std::vector<double> dist_traj_to_goal;
    // std::vector<geometry_msgs::PoseStamped>::iterator poseIt;
    // for (poseIt=global_plan_.begin(); poseIt < global_plan_.end(); poseIt++){
    //     geometry_msgs::PoseStamped& w = *poseIt;
    //     dist_traj_to_goal.push_back(euclidean_distance(x_end, w.pose.position.x, y_end, w.pose.position.y));
    // }

    // // get index of global plan point where distance is minimum to trajectory
    // double min_index = min_element(dist_traj_to_goal.begin(), dist_traj_to_goal.end()) - dist_traj_to_goal.begin();

    // cost = 100 * (1 - min_index/global_plan_.size());
    // //ROS_INFO("goal cost -- 1 -- %f", cost); // returns cost of last point on trajectory
    // return cost;
    // }
}

} // NAMESPACE