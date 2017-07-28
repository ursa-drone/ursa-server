/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
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
 * Author: TKruse
 *********************************************************************/

#include <ursa_local_planner/ursa_traj_gen.h>

#include <cmath>

#include <base_local_planner/velocity_iterator.h>

//DEBUG
#include <ros/ros.h>


namespace ursa_local_planner {

void UrsaTrajectoryGenerator::initialise(
  const Eigen::Vector3f& pos,
  const Eigen::Vector3f& vel,
  std::vector<geometry_msgs::PoseStamped> global_plan,
  base_local_planner::LocalPlannerLimits* limits,
  const Eigen::Vector3f& vsamples,
  std::vector<Eigen::Vector3f> additional_samples,
  bool discretize_by_time) {
  initialise(pos, vel, global_plan, limits, vsamples, discretize_by_time);
  // add static samples if any
  sample_params_.insert(sample_params_.end(), additional_samples.begin(), additional_samples.end());
}


void UrsaTrajectoryGenerator::initialise(
  const Eigen::Vector3f& pos,
  const Eigen::Vector3f& vel,
  std::vector<geometry_msgs::PoseStamped> global_plan,
  base_local_planner::LocalPlannerLimits* limits,
  const Eigen::Vector3f& vsamples,
  bool discretize_by_time) {
  /*
   * We actually generate all path sample vectors here, from which to generate trajectories later on
   */

  // Setup node handler and publishers
  ros::NodeHandle private_nh("~/");
  ros::NodeHandle nh;
  visualize_traj_gen_pub_ = private_nh.advertise<nav_msgs::Path>("visualize_traj_gen", 1);
  visualize_pose_ = private_nh.advertise<geometry_msgs::PoseStamped>("visualize_pose", 1);
  visualize_heading_ = private_nh.advertise<geometry_msgs::PoseStamped>("visualize_heading", 1);
  private_nh.getParam("global_costmap/robot_radius", robot_radius_);
  global_plan_ = global_plan;

  traj_gen_paths_.clear();

  // Clear sample points
  sample_params_.clear();
  next_sample_index_ = 0;
  pos_ = pos;
  double x_diff, y_diff, distance_sq;

  // Add in the first point from the global plan
  Eigen::Vector3f test_point = Eigen::Vector3f::Zero();
  test_point[0]=global_plan.front().pose.position.x;
  test_point[1]=global_plan.front().pose.position.y;
  test_point[2]=tf::getYaw(global_plan.front().pose.orientation);
  sample_params_.push_back(test_point);

  // Iterate over the remaining points and add if they are seperated by at least 10cm (make sure to add the last point)
  std::vector<geometry_msgs::PoseStamped>::iterator poseIt;
  for (poseIt=global_plan.begin()+1 ; poseIt < global_plan.end(); poseIt++){
  geometry_msgs::PoseStamped& w = *poseIt;
  x_diff = sample_params_.back()[0]-w.pose.position.x;
  y_diff = sample_params_.back()[1]-w.pose.position.y;
  distance_sq = x_diff*x_diff + y_diff*y_diff;
  if (distance_sq>=0.01 || poseIt == global_plan.end()){
    test_point[0]=w.pose.position.x;
    test_point[1]=w.pose.position.y;
    test_point[2]=tf::getYaw(w.pose.orientation);
    sample_params_.push_back(test_point);
  }
  }
}

void UrsaTrajectoryGenerator::setParameters(
  double sim_time,
  double sim_granularity,
  double angular_sim_granularity,
  bool use_dwa,
  double sim_period) {
  sim_time_ = sim_time;
  sim_granularity_ = sim_granularity;
  angular_sim_granularity_ = angular_sim_granularity;
  use_dwa_ = use_dwa;
  continued_acceleration_ = ! use_dwa_;
  sim_period_ = sim_period;
}

/**
 * Whether this generator can create more trajectories
 */
bool UrsaTrajectoryGenerator::hasMoreTrajectories() {
  return next_sample_index_ < sample_params_.size();
}

/**
 * Create and return the next sample trajectory
 */
bool UrsaTrajectoryGenerator::nextTrajectory(base_local_planner::Trajectory &comp_traj) {
  bool result = false;
  if (hasMoreTrajectories()) {
  if (generateTrajectory(
    pos_,
    vel_,
    sample_params_[next_sample_index_],
    comp_traj)) {
    result = true;
  }
  }
  next_sample_index_++;
  return result;
}

/**
 * @param pos current position of robot
 * @param vel unused
 * @param sample_target point on the global plan to generate trajectory
 */
bool UrsaTrajectoryGenerator::generateTrajectory(
    Eigen::Vector3f pos,
    Eigen::Vector3f vel,
    Eigen::Vector3f sample_target,
    base_local_planner::Trajectory& traj) {
  double eps = 1e-4;
  //traj.cost_   = -1.0; // placed here in case we return early
  //trajectory might be reused so we'll make sure to reset it
  traj.resetPoints();

  //how many steps do we need to simulate?
  int num_steps;
  double x_diff = sample_target[0]-pos[0];
  double y_diff = sample_target[1]-pos[1];
  double heading = fabs(atan(y_diff / x_diff));
  if ((x_diff >= 0) && (y_diff >= 0)) {
    heading = heading;
  }
  else if ((x_diff < 0) && (y_diff >= 0)){
    heading = M_PI - heading;
  }
  else if ((x_diff < 0) && (y_diff < 0)){
    heading = M_PI + heading;
  }else{
    heading = -heading;
  }
  double distance_sq = x_diff*x_diff+y_diff*y_diff;
  double distance=sqrt(distance_sq);
  num_steps = distance/0.1; //Parameterise this - currently 10cm
  if (num_steps==0) num_steps=1;


  // check if goal within robot radius
  double goal_x = global_plan_.back().pose.position.x;
  double goal_y = global_plan_.back().pose.position.y;

  if (  ((pos[0] - robot_radius_*1.5) <= goal_x) && 
        (goal_x < (pos[0] + robot_radius_*1.5))  &&
        ((pos[1] - robot_radius_*1.5) <= goal_y) && 
        (goal_y < (pos[1] + robot_radius_*1.5)) ) {
      heading = tf::getYaw(global_plan_.back().pose.orientation);      
    }
    else{
      heading = heading;
    }

    //create a path message
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = "map";
    pose.header.stamp = ros::Time::now();
    pose.pose.position.x = pos[0];
    pose.pose.position.y = pos[1];
    pose.pose.position.z = 0;
    pose.pose.orientation = tf::createQuaternionMsgFromYaw(pos[2]);
    visualize_pose_.publish(pose);

    //create a path message
    geometry_msgs::PoseStamped trajpose;
    trajpose.header.frame_id = "map";
    trajpose.header.stamp = ros::Time::now();
    trajpose.pose.position.x = pos[0];
    trajpose.pose.position.y = pos[1];
    trajpose.pose.position.z = 0;
    trajpose.pose.orientation = tf::createQuaternionMsgFromYaw(heading);
    visualize_heading_.publish(trajpose);





  //simulate the trajectory
  for (int i = 0; i < num_steps; ++i) {

  //add the point to the trajectory
  traj.addPoint(pos[0], pos[1], heading);

  pos[0]+=x_diff/num_steps;
  pos[1]+=y_diff/num_steps;

  } // end for simulation steps
  VisualiseTrajectoryGenerator(traj);


  return num_steps > 0; // true if trajectory has at least one point
}

void UrsaTrajectoryGenerator::VisualiseTrajectoryGenerator(base_local_planner::Trajectory& traj){
  double x; double y; double z;
  for(unsigned int i=0; i<traj.getPointsSize(); i++){
    geometry_msgs::PoseStamped pose;
    traj.getPoint(i, x, y, z);

    pose.header.stamp = ros::Time::now();
    pose.header.frame_id = "map";
    pose.pose.position.x = x;
    pose.pose.position.y = y;
    pose.pose.position.z = 0.3;

    traj_gen_paths_.push_back(pose);
  }

  base_local_planner::publishPlan(traj_gen_paths_, visualize_traj_gen_pub_);
}

} /* namespace base_local_planner */
