
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


bool UrsaGoalCostFunction::init(double penalty, std::vector<geometry_msgs::PoseStamped> global_plan){
  penalty_=penalty;
  global_plan_=global_plan;
}


double UrsaGoalCostFunction::scoreTrajectory(base_local_planner::Trajectory &traj) {
  //Setup
  double x_end, y_end, th_end;
  traj.getEndpoint(x_end, y_end, th_end);
  double x_diff, y_diff, distance_sq;

  //Iterate over the global plan - find a close point - use that to score
  int i=global_plan_.size();
  std::vector<geometry_msgs::PoseStamped>::iterator poseIt;
  for (poseIt=global_plan_.begin() ; poseIt < global_plan_.end(); poseIt++, i--){
    geometry_msgs::PoseStamped& w = *poseIt;
    x_diff = x_end-w.pose.position.x;
    y_diff = y_end-w.pose.position.y;
    distance_sq = x_diff*x_diff + y_diff*y_diff;
    if (distance_sq<=0.01){
      return i*penalty_;
    }
  }
  return -1.0;
}

} /* namespace ursa_local_planner */