
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


bool UrsaGoalCostFunction::init(double penalty, std::vector<geometry_msgs::PoseStamped> global_plan, tf::Stamped<tf::Pose> global_pose, double robot_radius){
    penalty_=penalty;
    global_plan_=global_plan;
    global_pose_=global_pose;
    robot_radius_=robot_radius;
}

double UrsaGoalCostFunction::globalPlanHeadingAtRadius() {
    // get global orientation at robot radius
    std::cout << "-------------" << std::endl;
    int i = 0;
    double global_heading_x = global_plan_[0].pose.position.x;
    double global_heading_y =  global_plan_[0].pose.position.y;

    std::cout << "robot_radius_" << robot_radius_ << std::endl;
    std::cout << "global_heading_x" << global_heading_x << std::endl;
    std::cout << "global_heading_y" << global_heading_y << std::endl;
    std::cout << "global_heading_y" << global_heading_y << std::endl;
    std::cout << "global_pose_.getOrigin().getX() - robot_radius_" << global_pose_.getOrigin().getX() - robot_radius_ << std::endl;
    std::cout << "global_pose_.getOrigin().getX() + robot_radius_" << global_pose_.getOrigin().getX() + robot_radius_ << std::endl;
       std::cout << "global_pose_.getOrigin().getY() - robot_radius_" << global_pose_.getOrigin().getY() - robot_radius_ << std::endl;
    std::cout << "global_pose_.getOrigin().getY() + robot_radius_" << global_pose_.getOrigin().getY() + robot_radius_ << std::endl;

    while   (((global_pose_.getOrigin().getX() - robot_radius_) <= global_heading_x) &&
            (global_heading_x < (global_pose_.getOrigin().getX() + robot_radius_))   &&
            ((global_pose_.getOrigin().getY() - robot_radius_) <= global_heading_y)  &&
            (global_heading_y < (global_pose_.getOrigin().getY() + robot_radius_))) {
                i++;
                global_heading_x = global_plan_[i].pose.position.x;
                global_heading_y =  global_plan_[i].pose.position.y;
            }
    std::cout << "@@i" << i << std::endl;
    std::cout << "-------------" << std::endl;
    return tf::getYaw(global_plan_[i].pose.orientation);
}

inline bool logically_equal(double a, double b, double error_factor=1.0)
{
  return std::abs(a-b)<1e-4;
}

double UrsaGoalCostFunction::scoreTrajectory(base_local_planner::Trajectory &traj) {
    //Setup
    double x_end, y_end, th_end;
    traj.getEndpoint(x_end, y_end, th_end);
    double x_diff, y_diff, distance_sq;

    // Get current pose and upper/lower theta range
    geometry_msgs::PoseStamped current_pose = global_plan_.front();
    double current_pose_x = global_pose_ .getOrigin().getX();
    double current_pose_y = global_pose_.getOrigin().getY();
    double current_pose_th = tf::getYaw(global_pose_.getRotation());
    double theta_lower = current_pose_th - M_PI/4;
    double theta_upper = current_pose_th + M_PI/4;

    double global_plan_at_radius_th = globalPlanHeadingAtRadius();
    std::cout << "global_plan_at_radius_th" << global_plan_at_radius_th << std::endl;

    // Set cost as length of path if not heading towards global plan
    // Set cost as inverse length if heading towards global plan
    std::cout << "tl - " << theta_lower << ", tu - " << theta_upper  << ", he - " << global_plan_at_radius_th << std::endl;
    std::cout << "check - " << ((theta_lower < global_plan_at_radius_th) && (global_plan_at_radius_th < theta_upper)) << std::endl;
    std::cout << "~check - " << !((theta_lower < global_plan_at_radius_th) && (global_plan_at_radius_th < theta_upper)) << std::endl;

    if (!((theta_lower < global_plan_at_radius_th) && (global_plan_at_radius_th < theta_upper))){
        // Lets give the trajectory that we want the lowest score...
        std::cout << "#################" << std::endl;
        std::cout << "x_end - " << x_end << std::endl;
        std::cout << "y_end - " << y_end << std::endl;
        std::cout << "th_end - " << th_end << std::endl;
        std::cout << "current_pose_x - " << current_pose_x << std::endl;
        std::cout << "current_pose_y - " << current_pose_y << std::endl;
        std::cout << "global_plan_at_radius_th - " << global_plan_at_radius_th << std::endl;
        std::cout << "global_plan_[10]" << global_plan_[10] << std::endl;
        std::cout << "x_end - current_pose_x" << x_end - current_pose_x << std::endl;
        std::cout << "logically_equal(x_end, current_pose_x)" << logically_equal(x_end, current_pose_x) << std::endl;
        std::cout << "y_end - current_pose_y" << y_end - current_pose_y << std::endl;
        std::cout << "logically_equal(y_end, current_pose_y)" << logically_equal(y_end, current_pose_y) << std::endl;
        std::cout << "th_end - global_plan_at_radius_th" << th_end - global_plan_at_radius_th << std::endl;
        std::cout << "logically_equal(th_end, global_plan_at_radius_th)" << logically_equal(th_end, global_plan_at_radius_th) << std::endl;
        std::cout << "#################" << std::endl;


        if (logically_equal(x_end, current_pose_x) && 
            logically_equal(y_end, current_pose_y) && 
            logically_equal(th_end, global_plan_at_radius_th)) {
                ROS_INFO("1.1");
                return 0;
            }
        else{
            ROS_INFO("1.2");
            return global_plan_.size();
            }
        }
    else{
        x_diff = x_end - current_pose_x;
        y_diff = y_end - current_pose_y;
        distance_sq = x_diff*x_diff + y_diff*y_diff;
        if (distance_sq <= DBL_EPSILON){
            ROS_INFO("2.1");
            return global_plan_.size();
            }
        else{
            ROS_INFO("2.2");
            return 1/distance_sq;
            }
        }
    }
}