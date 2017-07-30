
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
    // previous_result_traj_ = previous_result_traj;
}

void UrsaGoalCostFunction::loadPreviousLocalTraj(base_local_planner::Trajectory previous_result_traj){
    previous_result_traj_ = previous_result_traj;

    double previous_traj_x, previous_traj_y, previous_traj_th;
    if (previous_result_traj_.getPointsSize()){ // Check that there is something actually in there
        previous_result_traj_.getEndpoint(  previous_traj_x,
                                            previous_traj_y,
                                            previous_traj_th);
    }
}

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

bool UrsaGoalCostFunction::checkIfInsideRadius(double x, double y){
    // get global orientation at robot radius
    int i = 0;
    while   (((global_pose_.getOrigin().getX() - robot_radius_) <= x) &&
            (x < (global_pose_.getOrigin().getX() + robot_radius_))   &&
            ((global_pose_.getOrigin().getY() - robot_radius_) <= y)  &&
            (y < (global_pose_.getOrigin().getY() + robot_radius_))) {
                return 1;
            }
    return 0;
}

double UrsaGoalCostFunction::scoreTrajectory(base_local_planner::Trajectory &traj) {
    //Setup
    double x_end, y_end, th_end;
    traj.getEndpoint(x_end, y_end, th_end);
    double x_diff, y_diff, distance_sq;
    double cost = 0;

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

    // Prefer global plan
    if (logically_equal(x_end, goal_x) && 
        logically_equal(y_end, goal_y) && 
        logically_equal(th_end, goal_th)) {
        // ROS_INFO("0.0");
        return 0;
    }

    // Prefer previous plan if previous plan is outside robot radius
    double previous_traj_x, previous_traj_y, previous_traj_th;
    if (previous_result_traj_.getPointsSize()){ // Check that there is something actually in there
        previous_result_traj_.getEndpoint(  previous_traj_x,
                                            previous_traj_y,
                                            previous_traj_th);
    }

    if(checkIfInsideRadius(previous_traj_x, previous_traj_y)){ // Check if previous plan is inside radius
        // ROS_INFO("0.1");
    }else{
        if (logically_equal(x_end, previous_traj_x) && 
        logically_equal(y_end, previous_traj_y) && 
        logically_equal(th_end, previous_traj_th)) {
        ROS_INFO("0.2");
        return 0;
        }
    }

    // If not facing towards the global plan at the robot radius
    if (!((theta_lower < global_plan_at_radius_th) && (global_plan_at_radius_th < theta_upper))){
        // lowest score to trajectory at robot origin and in direction of global plan
        if (logically_equal(x_end, current_pose_x) && 
            logically_equal(y_end, current_pose_y) && 
            logically_equal(th_end, global_plan_at_radius_th)) {
                // ROS_INFO("1.1");
                return 1;
            }
        // higher score to all others
        else{
                // ROS_INFO("1.2");
                return 2;
            }
        }
    // Else if heading in the general direction of global plan...
    else{
        // largest score to point at robot origin
        x_diff = x_end - current_pose_x;
        y_diff = y_end - current_pose_y;
        distance_sq = x_diff*x_diff + y_diff*y_diff;
        if (distance_sq <= DBL_EPSILON){
            // ROS_INFO("2.1");
            return 1 + global_plan_.size(); // +1 to ensure global plan end point is always favoured
            }
        // smallest score to points further away
        else{
            // ROS_INFO("2.2");
            return 1 + 1/distance_sq;       // +1 to ensure global plan end point is always favoured
            }
        }
    }
}



        // // If goal is inside robot radius then the local plan orientation should be = goal orientation
        // if (((pos[0] - robot_radius_*1.5) <= goal_x) &&
        //         (goal_x < (pos[0] + robot_radius_*1.5))  &&
        //         ((pos[1] - robot_radius_*1.5) <= goal_y) &&
        //         (goal_y < (pos[1] + robot_radius_*1.5))) {
        //                 heading = tf::getYaw(global_plan_.back().pose.orientation);
        //                 traj.addPoint(goal_x, goal_y, heading);
        //                 ROS_INFO("#### 1 ");
        // }

        // std::cout << "#################" << std::endl;
        // std::cout << "x_end - " << x_end << std::endl;
        // std::cout << "y_end - " << y_end << std::endl;
        // std::cout << "th_end - " << th_end << std::endl;
        // std::cout << "current_pose_x - " << current_pose_x << std::endl;
        // std::cout << "current_pose_y - " << current_pose_y << std::endl;
        // std::cout << "global_plan_at_radius_th - " << global_plan_at_radius_th << std::endl;
        // std::cout << "global_plan_[10]" << global_plan_[10] << std::endl;
        // std::cout << "x_end - current_pose_x" << x_end - current_pose_x << std::endl;
        // std::cout << "logically_equal(x_end, current_pose_x)" << logically_equal(x_end, current_pose_x) << std::endl;
        // std::cout << "y_end - current_pose_y" << y_end - current_pose_y << std::endl;
        // std::cout << "logically_equal(y_end, current_pose_y)" << logically_equal(y_end, current_pose_y) << std::endl;
        // std::cout << "th_end - global_plan_at_radius_th" << th_end - global_plan_at_radius_th << std::endl;
        // std::cout << "logically_equal(th_end, global_plan_at_radius_th)" << logically_equal(th_end, global_plan_at_radius_th) << std::endl;
        // std::cout << "#################" << std::endl;


    // std::cout << "global_plan_at_radius_th" << global_plan_at_radius_th << std::endl;

    // // Set cost as length of path if not heading towards global plan
    // // Set cost as inverse length if heading towards global plan
    // std::cout << "tl - " << theta_lower << ", tu - " << theta_upper  << ", he - " << global_plan_at_radius_th << std::endl;
    // std::cout << "check - " << ((theta_lower < global_plan_at_radius_th) && (global_plan_at_radius_th < theta_upper)) << std::endl;
    // std::cout << "~check - " << !((theta_lower < global_plan_at_radius_th) && (global_plan_at_radius_th < theta_upper)) << std::endl;

    // std::cout << "robot_radius_" << robot_radius_ << std::endl;
    // std::cout << "global_heading_x" << global_heading_x << std::endl;
    // std::cout << "global_heading_y" << global_heading_y << std::endl;
    // std::cout << "global_heading_y" << global_heading_y << std::endl;
    // std::cout << "global_pose_.getOrigin().getX() - robot_radius_" << global_pose_.getOrigin().getX() - robot_radius_ << std::endl;
    // std::cout << "global_pose_.getOrigin().getX() + robot_radius_" << global_pose_.getOrigin().getX() + robot_radius_ << std::endl;
    //    std::cout << "global_pose_.getOrigin().getY() - robot_radius_" << global_pose_.getOrigin().getY() - robot_radius_ << std::endl;
    // std::cout << "global_pose_.getOrigin().getY() + robot_radius_" << global_pose_.getOrigin().getY() + robot_radius_ << std::endl;

    // std::cout << "-------------" << std::endl;
    // std::cout << "@@i" << i << std::endl;
    // std::cout << "-------------" << std::endl;