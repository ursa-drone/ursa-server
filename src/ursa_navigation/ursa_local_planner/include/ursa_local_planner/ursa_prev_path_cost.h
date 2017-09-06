
/*********************************************************************
 * Author: Isaac Hook
 *********************************************************************/

#ifndef URSA_PREV_PATH_COST_FUNCTION_H_
#define URSA_PREV_PATH_COST_FUNCTION_H_

#include <base_local_planner/goal_functions.h>
#include <base_local_planner/trajectory_cost_function.h>

namespace ursa_local_planner {

class UrsaPrevPathCostFunction: public base_local_planner::TrajectoryCostFunction {
public:
  UrsaPrevPathCostFunction() {}
  ~UrsaPrevPathCostFunction() {}

  double scoreTrajectory(base_local_planner::Trajectory &traj);

  bool prepare() {return true;};

  bool init(base_local_planner::Trajectory previous_result_traj);

  void setPenalty(double penalty) {penalty_ = penalty;}

private:
    double penalty_;
    base_local_planner::Trajectory previous_result_traj_;
};

} /* namespace ursa_local_planner */
#endif /* URSA_PREV_PATH_COST_FUNCTION_H_ */