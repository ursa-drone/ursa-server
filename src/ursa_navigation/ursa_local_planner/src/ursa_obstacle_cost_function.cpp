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

#include <ursa_local_planner/ursa_obstacle_cost_function.h>
#include <cmath>
#include <Eigen/Core>
#include <ros/console.h>
#include <costmap_2d/cost_values.h>
#include <iostream>
using namespace std;
using namespace costmap_2d;

namespace ursa_local_planner {

UrsaObstacleCostFunction::UrsaObstacleCostFunction(costmap_2d::Costmap2D* costmap) 
    : costmap_(costmap), sum_scores_(false) {
  if (costmap != NULL) {
    world_model_ = new ursa_local_planner::UrsaCostmapModel(*costmap_);
  }
}

UrsaObstacleCostFunction::~UrsaObstacleCostFunction() {
  if (world_model_ != NULL) {
    delete world_model_;
  }
}

void UrsaObstacleCostFunction::setParams(double max_trans_vel, double max_scaling_factor, double scaling_speed) {
  // TODO: move this to prepare if possible
  max_trans_vel_ = max_trans_vel;
  max_scaling_factor_ = max_scaling_factor;
  scaling_speed_ = scaling_speed;
}

void UrsaObstacleCostFunction::setFootprint(std::vector<geometry_msgs::Point> footprint_spec) {
  footprint_spec_ = footprint_spec;
}

bool UrsaObstacleCostFunction::prepare() {
  return true;
}

inline bool checkIfInsideRadius(double x, double y, double pose_x, double pose_y, double multiplier, double radius){
    // get global orientation at robot radius
    int i = 0;
    while   (((pose_x - radius*multiplier) <= x) &&
             (x < (pose_x + radius*multiplier))  &&
             ((pose_y - radius*multiplier) <= y) &&
             (y < (pose_y + radius*multiplier))) {
                return 1;
            }
    return 0;
}

double UrsaObstacleCostFunction::scoreTrajectory(base_local_planner::Trajectory &traj) {
  double cost = 0;
  double scale = getScalingFactor(traj, scaling_speed_, max_trans_vel_, max_scaling_factor_);
  double px, py, pth;
  if (footprint_spec_.size() == 0) {
    // Bug, should never happen
    ROS_ERROR("Footprint spec is empty, maybe missing call to setFootprint?");
    return -9;
  }

  double dx, dy, dth;
  traj.getPoint(0, dx, dy, dth);
  for (unsigned int i = 0; i < traj.getPointsSize(); ++i) {
    traj.getPoint(i, px, py, pth);
    double f_cost = footprintCost(px, py, pth,
        scale, footprint_spec_,
        costmap_, world_model_);

    if(f_cost < 0){
        return f_cost;
    }
    // return the cost of a point just outside the radius of drone
    if (checkIfInsideRadius(px,py,dx,dy,1,footprint_spec_[0].x)){
      cost=f_cost;
      continue;
      //ROS_INFO("obstacle cost -- 1 -- %f", f_cost); // returns cost of last point on trajectory
      //return f_cost;
    }
    // if(sum_scores_)
    //     cost +=  f_cost;
    // else
    //     cost = f_cost;
    // if (f_cost > cost){
    //   cost = f_cost;      
    // }
    if (f_cost>cost){
      cost=f_cost;
    }
    
  }
  // cost = cost / traj.getPointsSize();
  // ROS_INFO("obstacle cost -- %f", cost); // returns cost of last point on trajectory
  return cost;
}

double UrsaObstacleCostFunction::getScalingFactor(base_local_planner::Trajectory &traj, double scaling_speed, double max_trans_vel, double max_scaling_factor) {
  double vmag = hypot(traj.xv_, traj.yv_);

  //if we're over a certain speed threshold, we'll scale the robot's
  //footprint to make it either slow down or stay further from walls
  double scale = 1.0;
  if (vmag > scaling_speed) {
    //scale up to the max scaling factor linearly... this could be changed later
    double ratio = (vmag - scaling_speed) / (max_trans_vel - scaling_speed);
    scale = max_scaling_factor * ratio + 1.0;
  }
  return scale;
}

double UrsaObstacleCostFunction::footprintCost (
    const double& x,
    const double& y,
    const double& th,
    double scale,
    std::vector<geometry_msgs::Point> footprint_spec,
    costmap_2d::Costmap2D* costmap,
    base_local_planner::WorldModel* world_model) {

  //check if the footprint is legal
  // TODO: Cache inscribed radius
  //double footprint_cost = world_model->footprintCost(x, y, th, footprint_spec);

  unsigned int cell_x, cell_y;
  //we won't allow trajectories that go off the map... shouldn't happen that often anyways
  if ( ! costmap->worldToMap(x, y, cell_x, cell_y)) {
    return -7.0;
  }
  double footprint_cost = costmap->getCost(cell_x, cell_y);
  if (footprint_cost < 0) {
    return -6.0;
  }
  //if(cost == LETHAL_OBSTACLE || cost == INSCRIBED_INFLATED_OBSTACLE)
  if(footprint_cost == LETHAL_OBSTACLE || footprint_cost == INSCRIBED_INFLATED_OBSTACLE || footprint_cost == NO_INFORMATION)
    return -1.0;
  return footprint_cost;

  // return the max of (cell cost at footprint (last two points)) and (cell cost at centre of robot)
  // double occ_cost = std::max(std::max(0.0, footprint_cost), double(costmap->getCost(cell_x, cell_y)));

  // return occ_cost;

}

} /* namespace ursa_local_planner */
