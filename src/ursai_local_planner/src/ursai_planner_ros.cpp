/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2009, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of Willow Garage, Inc. nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Author: Eitan Marder-Eppstein
*********************************************************************/
// Isaac's additional headers
#include <iostream>
#include "ros/ros.h"
#include "std_msgs/String.h"

#include <ursai_local_planner/ursai_planner_ros.h>
#include <Eigen/Core>
#include <cmath>

#include <ros/console.h>

#include <pluginlib/class_list_macros.h>

#include <base_local_planner/goal_functions.h>
#include <nav_msgs/Path.h>

//register this planner as a BaseLocalPlanner plugin
PLUGINLIB_EXPORT_CLASS(ursai_local_planner::URSAIPlannerROS, nav_core::BaseLocalPlanner)

using namespace std;

namespace ursai_local_planner {

  void URSAIPlannerROS::reconfigureCB(URSAIPlannerConfig &config, uint32_t level) {
      // cout << "@@@@@URSAIPlannerROS::reconfigureCB" << endl;
      if (setup_ && config.restore_defaults) {
        config = default_config_;
        config.restore_defaults = false;
      }
      if ( ! setup_) {
        // cout << "@@@@@----setup_: " << setup_ << endl;
        // cout << "@@@@@----default_config_.sim_granularity: " << default_config_.sim_granularity << endl;
        default_config_ = config;
        // cout << "@@@@@----default_config_.sim_granularity: " << default_config_.sim_granularity << endl;
        setup_ = true; // set to true, so default config is not overwritten
      }

      // update generic local planner params
      base_local_planner::LocalPlannerLimits limits;
      limits.max_trans_vel = config.max_trans_vel;
      limits.min_trans_vel = config.min_trans_vel;
      limits.max_vel_x = config.max_vel_x;
      limits.min_vel_x = config.min_vel_x;
      limits.max_vel_y = config.max_vel_y;
      limits.min_vel_y = config.min_vel_y;
      limits.max_rot_vel = config.max_rot_vel;
      limits.min_rot_vel = config.min_rot_vel;
      limits.acc_lim_x = config.acc_lim_x;
      limits.acc_lim_y = config.acc_lim_y;
      limits.acc_lim_theta = config.acc_lim_theta;
      limits.acc_limit_trans = config.acc_limit_trans;
      limits.xy_goal_tolerance = config.xy_goal_tolerance;
      limits.yaw_goal_tolerance = config.yaw_goal_tolerance;
      limits.prune_plan = config.prune_plan;
      limits.trans_stopped_vel = config.trans_stopped_vel;
      limits.rot_stopped_vel = config.rot_stopped_vel;
      planner_util_.reconfigureCB(limits, config.restore_defaults);

      // update ursai specific configuration
      dp_->reconfigure(config);
  }

  URSAIPlannerROS::URSAIPlannerROS() : initialized_(false),
      odom_helper_("odom"), setup_(false) { // setup = false so that that reconfigure assigns the configuration file to default_config_
      // cout << "@@@@@URSAIPlannerROS::URSAIPlannerROS" << endl;
  }

  void URSAIPlannerROS::initialize(
      std::string name,
      tf::TransformListener* tf,
      costmap_2d::Costmap2DROS* costmap_ros) {
      // cout << "@@@@@URSAIPlannerROS::initialize" << endl;
      // cout << "@@@@@----name: " << name << endl;
      // cout << "@@@@@----tf: " << tf->allFramesAsDot() << endl;
      // cout << "@@@@@----costmap_ros: " << costmap_ros->getGlobalFrameID() << endl;

    if (! isInitialized()) {


      // roscpp's interface for creating publishers, subscribers, etc....
      ros::NodeHandle private_nh("~/" + name); 


      // create a local and global publisher
      g_plan_pub_ = private_nh.advertise<nav_msgs::Path>("global_plan", 1); // returns a publisher allowing you to publish a message ona topic
                                                                            // <message type>(topic, queue size, latch=when new subscribers, get last message)
      // cout << "@@@@@----g_plan_pub_: " << g_plan_pub_.getTopic() << endl;

      l_plan_pub_ = private_nh.advertise<nav_msgs::Path>("local_plan", 1);
      // cout << "@@@@@----l_plan_pub_: " << l_plan_pub_.getTopic() << endl;

      // assign some names to our tf and costmap pointers
      tf_ = tf; // tf::TransformListener* tf
      // cout << "@@@@@----tf_: " << tf_->allFramesAsDot() << endl;
      costmap_ros_ = costmap_ros; // costmap_2d::Costmap2DROS* costmap_ros
      // cout << "@@@@@----costmap_ros_: " << costmap_ros_->getGlobalFrameID() << endl;
      costmap_ros_->getRobotPose(current_pose_);  // bool  getRobotPose (tf::Stamped< tf::Pose > &global_pose) const
                                                  // std::array<int, 5> n; -- simple declaration of an array, n is standard array of type int and length 5
                                                  // std::vector<int> marks; -- declaration of a vector named marks and type int
                                                  // tf::Stamped< tf::Pose > -- declaration of tf::Stamped of type tf::Pose
                                                  // template class
      // cout << "@@@@@----current_pose_.frame_id_: " << current_pose_.frame_id_ << endl;
      // cout << "@@@@@----current_pose_.stamp_: " << current_pose_.stamp_ << endl;
      // cout << "@@@@@----costmap_ros_->getRobotPose(current_pose_): " << costmap_ros_->getRobotPose(current_pose_) << endl;

      // make sure to update the costmap we'll use for this cycle
      costmap_2d::Costmap2D* costmap = costmap_ros_->getCostmap(); // give a name to the most recent costmap

      // initialise planner
      planner_util_.initialize(tf, costmap, costmap_ros_->getGlobalFrameID()); // void  initialize (tf::TransformListener *tf, costmap_2d::Costmap2D *costmap, std::string global_frame)

      //create the actual planner that we'll use.. it'll configure itself from the parameter server
        // creates a pointer to   URSAIPlanner::URSAIPlanner(std::string name, base_local_planner::LocalPlannerUtil *planner_util) from ursai_planner.cpp
        // this function uses the name and config file &planner_util_ to intialise
        // member functions initialised are 
                // planner_util_(planner_util), // need to check header file to see the definition of all these variables
                // obstacle_costs_(planner_util->getCostmap()), -- returns scoreTrajectory when given a trajectory
                // path_costs_(planner_util->getCostmap()),  -- from MapGridCostFunction, returns a bunch of things when costmap received: cell costs, obstacele costs...
                // goal_costs_(planner_util->getCostmap(), 0.0, 0.0, true), -- also from MapGridCostFunction
                // goal_front_costs_(planner_util->getCostmap(), 0.0, 0.0, true), -- also from MapGridCostFunction
                // alignment_costs_(planner_util->getCostmap()) -- also from MapGridCostFunction
      dp_ = boost::shared_ptr<URSAIPlanner>(new URSAIPlanner(name, &planner_util_));

      if( private_nh.getParam( "odom_topic", odom_topic_ ))
      {
        odom_helper_.setOdomTopic( odom_topic_ );
      }
      
      initialized_ = true;

      dsrv_ = new dynamic_reconfigure::Server<URSAIPlannerConfig>(private_nh);
      dynamic_reconfigure::Server<URSAIPlannerConfig>::CallbackType cb = boost::bind(&URSAIPlannerROS::reconfigureCB, this, _1, _2);
      dsrv_->setCallback(cb);
    }
    else{
      ROS_WARN("This planner has already been initialized, doing nothing.");
    }
  }
  
  bool URSAIPlannerROS::setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan) {
      // cout << "@@@@@URSAIPlannerROS::setPlan" << endl;
    
    if (! isInitialized()) {
      ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
      return false;
    }
    //when we get a new plan, we also want to clear any latch we may have on goal tolerances
    latchedStopRotateController_.resetLatching();

    ROS_INFO("Got new plan");
    return dp_->setPlan(orig_global_plan);
  }

  bool URSAIPlannerROS::isGoalReached() {
      // cout << "@@@@@URSAIPlannerROS::isGoalReached" << endl;
      // cout << "@@@@@----URSAIPlannerROS::isGoalReached(return) = " << latchedStopRotateController_.isGoalReached(&planner_util_, odom_helper_, current_pose_) << endl;

    if (! isInitialized()) {
      ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
      return false;
    }
    if ( ! costmap_ros_->getRobotPose(current_pose_)) {
      ROS_ERROR("Could not get robot pose");
      return false;
    }

    if(latchedStopRotateController_.isGoalReached(&planner_util_, odom_helper_, current_pose_)) {
      ROS_INFO("Goal reached");
      return true;
    } else {
      return false;
    }
  }

  void URSAIPlannerROS::publishLocalPlan(std::vector<geometry_msgs::PoseStamped>& path) {
      // cout << "@@@@@URSAIPlannerROS::publishLocalPlan" << endl;
      // for(unsigned int i=0; i < path.size(); i++){
        // cout << "@@@@@----path[" << i << "] = " << path[i] << endl;
      // }
    base_local_planner::publishPlan(path, l_plan_pub_);
    // Publish a plan for visualization purposes.
            // path  The plan to publish
            // l_plan_pub_ is the thing we defined above - just means the local_plan topic (/move_base/URSAIPlannerROS/local_plan)
            // basically publishes the path on the local plan topic which is of message type >> nav_msgs/Path Message
            // for visualisation purposes
  }


  void URSAIPlannerROS::publishGlobalPlan(std::vector<geometry_msgs::PoseStamped>& path) {
      // cout << "@@@@@URSAIPlannerROS::publishGlobalPlan" << endl;
      // for(unsigned int i=0; i < path.size(); i++){
        // cout << "@@@@@----path[" << i << "] = " << path[i] << endl;
      // }
    base_local_planner::publishPlan(path, g_plan_pub_);
  }

  URSAIPlannerROS::~URSAIPlannerROS(){
      // cout << "@@@@@URSAIPlannerROS::~URSAIPlannerROS" << endl;
    //make sure to clean things up
    delete dsrv_;
  }



  bool URSAIPlannerROS::ursaiComputeVelocityCommands(tf::Stamped<tf::Pose> &global_pose, geometry_msgs::Twist& cmd_vel) {
      // cout << "@@@@@URSAIPlannerROS::ursaiComputeVelocityCommands" << endl;
    // dynamic window sampling approach to get useful velocity commands
    if(! isInitialized()){
      ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
      return false;
    }

    tf::Stamped<tf::Pose> robot_vel;
    odom_helper_.getRobotVel(robot_vel);

    /* For timing uncomment
    struct timeval start, end;
    double start_t, end_t, t_diff;
    gettimeofday(&start, NULL);+
    */

    //compute what trajectory to drive along
    tf::Stamped<tf::Pose> drive_cmds;
    drive_cmds.frame_id_ = costmap_ros_->getBaseFrameID();
    
    // call with updated footprint
      // I feel like this is pretty important
      // In the tradish implementation of the local planner cmd_vel topic would be published after best path is found
      // not sure how cmd_vel gets published though
    base_local_planner::Trajectory path = dp_->findBestPath(global_pose, robot_vel, drive_cmds, costmap_ros_->getRobotFootprint());
    //ROS_ERROR("Best: %.2f, %.2f, %.2f, %.2f", path.xv_, path.yv_, path.thetav_, path.cost_);

    /* For timing uncomment
    gettimeofday(&end, NULL);
    start_t = start.tv_sec + double(start.tv_usec) / 1e6;
    end_t = end.tv_sec + double(end.tv_usec) / 1e6;
    t_diff = end_t - start_t;
    ROS_INFO("Cycle time: %.9f", t_diff);
    */

    //pass along drive commands
    cmd_vel.linear.x = drive_cmds.getOrigin().getX();
    cmd_vel.linear.y = drive_cmds.getOrigin().getY();
    cmd_vel.angular.z = tf::getYaw(drive_cmds.getRotation());

    //if we cannot move... tell someone
    std::vector<geometry_msgs::PoseStamped> local_plan;
    if(path.cost_ < 0) {
      ROS_DEBUG_NAMED("ursai_local_planner",
          "The ursai local planner failed to find a valid plan, cost functions discarded all candidates. This can mean there is an obstacle too close to the robot.");
      local_plan.clear();
      publishLocalPlan(local_plan);
      return false;
    }

    ROS_DEBUG_NAMED("ursai_local_planner", "A valid velocity command of (%.2f, %.2f, %.2f) was found for this cycle.", 
                    cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z);

    // Fill out the local plan
    for(unsigned int i = 0; i < path.getPointsSize(); ++i) {
      double p_x, p_y, p_th;
      path.getPoint(i, p_x, p_y, p_th);

      tf::Stamped<tf::Pose> p =
              tf::Stamped<tf::Pose>(tf::Pose(
                      tf::createQuaternionFromYaw(p_th),
                      tf::Point(p_x, p_y, 0.0)),
                      ros::Time::now(),
                      costmap_ros_->getGlobalFrameID());
      geometry_msgs::PoseStamped pose;
      tf::poseStampedTFToMsg(p, pose);
      local_plan.push_back(pose);
    }

    //publish information to the visualizer

    publishLocalPlan(local_plan);
    return true;
  }




  bool URSAIPlannerROS::computeVelocityCommands(geometry_msgs::Twist& cmd_vel) {
      // cout << "@@@@@URSAIPlannerROS::computeVelocityCommands" << endl;
    // dispatches to either ursai sampling control or stop and rotate control, depending on whether we have been close enough to goal
    if ( ! costmap_ros_->getRobotPose(current_pose_)) {
      ROS_ERROR("Could not get robot pose");
      return false;
    }
    std::vector<geometry_msgs::PoseStamped> transformed_plan;
    if ( ! planner_util_.getLocalPlan(current_pose_, transformed_plan)) {
      ROS_ERROR("Could not get local plan");
      return false;
    }

    //if the global plan passed in is empty... we won't do anything
    if(transformed_plan.empty()) {
      ROS_WARN_NAMED("ursai_local_planner", "Received an empty transformed plan.");
      return false;
    }
    ROS_DEBUG_NAMED("ursai_local_planner", "Received a transformed plan with %zu points.", transformed_plan.size());

    // update plan in ursai_planner even if we just stop and rotate, to allow checkTrajectory
    dp_->updatePlanAndLocalCosts(current_pose_, transformed_plan);

    if (latchedStopRotateController_.isPositionReached(&planner_util_, current_pose_)) {
      //publish an empty plan because we've reached our goal position
      std::vector<geometry_msgs::PoseStamped> local_plan;
      std::vector<geometry_msgs::PoseStamped> transformed_plan;
      publishGlobalPlan(transformed_plan);
      publishLocalPlan(local_plan);
      base_local_planner::LocalPlannerLimits limits = planner_util_.getCurrentLimits();
      return latchedStopRotateController_.computeVelocityCommandsStopRotate(
          cmd_vel,
          limits.getAccLimits(),
          dp_->getSimPeriod(),
          &planner_util_,
          odom_helper_,
          current_pose_,
          boost::bind(&URSAIPlanner::checkTrajectory, dp_, _1, _2, _3));
    } else {
      bool isOk = ursaiComputeVelocityCommands(current_pose_, cmd_vel);
      if (isOk) {
        publishGlobalPlan(transformed_plan);
      } else {
        ROS_WARN_NAMED("ursai_local_planner", "URSAI planner failed to produce path.");
        std::vector<geometry_msgs::PoseStamped> empty_plan;
        publishGlobalPlan(empty_plan);
      }
      return isOk;
    }
  }


};
