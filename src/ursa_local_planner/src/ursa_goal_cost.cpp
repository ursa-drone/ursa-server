
/*
 * prefer_forward_cost_function.cpp
 *
 *  Created on: Apr 4, 2012
 *      Author: tkruse
 */

#include <ursa_local_planner/ursa_goal_cost.h>
//DEBUG
#include <ros/ros.h>

#include <math.h>

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
bool UrsaGoalCostFunction::reconfigure(double ucfg){
    ucfg_ = ucfg;
};

double UrsaGoalCostFunction::globalPlanHeadingAtRadius() {
    // get global orientation at robot radius
    int i = 0;
    double global_heading_x = global_plan_[0].pose.position.x;
    double global_heading_y =  global_plan_[0].pose.position.y;

    while   (((global_pose_.getOrigin().getX() - robot_radius_) <= global_heading_x) &&
            (global_heading_x < (global_pose_.getOrigin().getX() + robot_radius_))   &&
            ((global_pose_.getOrigin().getY() - robot_radius_) <= global_heading_y)  &&
            (global_heading_y < (global_pose_.getOrigin().getY() + robot_radius_))) {
                i++;
                global_heading_x = global_plan_[i].pose.position.x;
                global_heading_y =  global_plan_[i].pose.position.y;
            }

    return tf::getYaw(global_plan_[i].pose.orientation);
}

inline bool logically_equal(double a, double b, double error_factor=1.0)
{
  return std::abs(a-b)<1e-4;
}

inline double euclidean_distance(double x1, double x2, double y1, double y2)
{
    double x_diff = x1 - x2;
    double y_diff = y1 - y2;
    double distance_sq = x_diff*x_diff + y_diff*y_diff;
    double dist = sqrt(distance_sq);

    return dist;
}

bool UrsaGoalCostFunction::checkIfInsideRadius(double x, double y, double multiplier){
    // get global orientation at robot radius
    int i = 0;
    while   (((global_pose_.getOrigin().getX() - (robot_radius_ * multiplier)) <= x) &&
            (x < (global_pose_.getOrigin().getX() + (robot_radius_ * multiplier)))   &&
            ((global_pose_.getOrigin().getY() - (robot_radius_ * multiplier)) <= y)  &&
            (y < (global_pose_.getOrigin().getY() + (robot_radius_) * multiplier))) {
                return 1;
            }
    return 0;
}

double UrsaGoalCostFunction::scoreTrajectory(base_local_planner::Trajectory &traj) {
    //Setup
    double cost;
    const int MAX_COST = 253;
    double x_end, y_end, th_end;
    traj.getEndpoint(x_end, y_end, th_end);
    double x_diff, y_diff, distance_sq, dist;

    // Get current pose and upper/lower theta range
    geometry_msgs::PoseStamped current_pose = global_plan_.front();
    double current_pose_x = global_pose_ .getOrigin().getX();
    double current_pose_y = global_pose_.getOrigin().getY();
    double current_pose_th = tf::getYaw(global_pose_.getRotation());
    double theta_lower = current_pose_th - M_PI/4;
    double theta_upper = current_pose_th + M_PI/4;
    double global_plan_at_radius_th = globalPlanHeadingAtRadius();
    double goal_x = global_plan_.back().pose.position.x;
    double goal_y = global_plan_.back().pose.position.y;
    double goal_th = tf::getYaw(global_plan_.back().pose.orientation);

    // Prefer global plan -- for the local trajectory that has the goal as its end point
    if (logically_equal(x_end, goal_x)          &&
        logically_equal(y_end, goal_y)          &&
        logically_equal(th_end, goal_th)) {
        if (checkIfInsideRadius(goal_x, goal_y, 2)) { // but only give low cost if nearby
            // ROS_INFO("scoreTrajectory cost -- %f", 0.);
            ROS_INFO("### 1 -- 0");
            return 0;
        }
        else{
            // ROS_INFO("scoreTrajectory cost -- %d", MAX_COST);
            ROS_INFO("### 2 -- %d", MAX_COST);
            return MAX_COST; // otherwise give max
        }
    }

    // If not facing towards the global plan at the robot radius
    if (!((theta_lower < global_plan_at_radius_th) && (global_plan_at_radius_th < theta_upper))){
        // lowest score to trajectory at robot origin and in direction of global plan
        if (logically_equal(x_end, current_pose_x) && 
            logically_equal(y_end, current_pose_y) && 
            logically_equal(th_end, global_plan_at_radius_th)) {
                // ROS_INFO("scoreTrajectory cost -- %f", 1.);
                ROS_INFO("### 3 -- 1");
                return 1;
            }
        // higher score to all others
        else{
                // ROS_INFO("scoreTrajectory cost -- %f", 2.);
                ROS_INFO("### 4 -- 2");
                return 2;
            }
        }
    // Else if heading in the general direction of global plan...
    else{

        double dist_traj_to_goal = euclidean_distance(x_end, goal_x, y_end, goal_y);
        double dist_pose_to_goal = euclidean_distance(current_pose_x, goal_x, current_pose_y, goal_y);
        cost = 10 * (dist_traj_to_goal/dist_pose_to_goal);
        // ROS_INFO("x_end, %f, goal_x %f, y_end %f, goal_y %f, dist_traj_to_goal %f", x_end, goal_x, y_end, goal_y, dist_traj_to_goal);
        // ROS_INFO("current_pose_x, %f, goal_x %f, current_pose_y %f, goal_y %f, dist_pose_to_goal %f", current_pose_x, goal_x, current_pose_y, goal_y, dist_pose_to_goal);
        ROS_INFO("### 5 -- %f", cost);
        return cost;
        }
    }
}