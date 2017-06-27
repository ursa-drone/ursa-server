
/*********************************************************************
 * Author: Lachlan Dowling
 *********************************************************************/

#ifndef URSA_GOAL_COST_FUNCTION_H_
#define URSA_GOAL_COST_FUNCTION_H_

#include <base_local_planner/goal_functions.h>
#include <base_local_planner/trajectory_cost_function.h>

namespace ursa_local_planner {

class UrsaGoalCostFunction: public base_local_planner::TrajectoryCostFunction {
public:

  UrsaGoalCostFunction() {}
  ~UrsaGoalCostFunction() {}

  double scoreTrajectory(base_local_planner::Trajectory &traj);

  bool prepare() {return true;};

  bool init(double penalty, std::vector<geometry_msgs::PoseStamped> global_plan);

  void setPenalty(double penalty) {
    penalty_ = penalty;
  }

private:
  double penalty_;
  std::vector<geometry_msgs::PoseStamped> global_plan_;
};

} /* namespace ursa_local_planner */
#endif /* URSA_GOAL_COST_FUNCTION_H_ */