
/*********************************************************************
 * Author: Lachlan Dowling
 *********************************************************************/

#ifndef URSA_STICKY_COST_FUNCTION_H_
#define URSA_STICKY_COST_FUNCTION_H_

#include <base_local_planner/goal_functions.h>
#include <base_local_planner/trajectory_cost_function.h>

namespace ursa_local_planner {

class UrsaStickyCostFunction: public base_local_planner::TrajectoryCostFunction {
public:
  UrsaStickyCostFunction() {}
  ~UrsaStickyCostFunction() {}

  double scoreTrajectory(base_local_planner::Trajectory &traj);

  bool prepare() {return true;};

  bool init(double penalty,
            std::vector<geometry_msgs::PoseStamped> global_plan,
            tf::Stamped<tf::Pose> global_pose, 
            double robot_radius,
            base_local_planner::Trajectory previous_result_traj);

  void setPenalty(double penalty) {
    penalty_ = penalty;
  }

private:
    double penalty_;
    tf::Stamped<tf::Pose> global_pose_;
    std::vector<geometry_msgs::PoseStamped> global_plan_;
    double robot_radius_;
    base_local_planner::Trajectory previous_traj_;
};

} /* namespace ursa_local_planner */
#endif /* URSA_STICKY_COST_FUNCTION_H_ */


  // double globalPlanHeadingAtRadius();