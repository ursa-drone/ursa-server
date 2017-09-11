/*********************************************************************
* Rotate recovery behaviour for URSA drone
*
* Author: LAOSAAC
*********************************************************************/
#ifndef URSA_ROTATE_RECOVERY_H_
#define URSA_ROTATE_RECOVERY_H_
#include <nav_core/recovery_behavior.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <tf/transform_listener.h>
#include <ros/ros.h>
#include <base_local_planner/costmap_model.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <angles/angles.h>


namespace ursa_rotate_recovery{
  /**
   * @class RotateRecovery
   * @brief A recovery behavior that rotates the robot 180 degrees in order to clear out space
   */
  class RotateRecovery : public nav_core::RecoveryBehavior {
    public:
      /**
       * @brief  Constructor, make sure to call initialize in addition to actually initialize the object
       * @param  
       * @return 
       */
      RotateRecovery();

      /**
       * @brief  Initialization function for the RotateRecovery recovery behavior
       * @param tf A pointer to a transform listener
       * @param global_costmap A pointer to the global_costmap used by the navigation stack 
       * @param local_costmap A pointer to the local_costmap used by the navigation stack 
       */
      void initialize(std::string name, tf::TransformListener* tf, 
          costmap_2d::Costmap2DROS* global_costmap, costmap_2d::Costmap2DROS* local_costmap);

      /**
       * @brief  Run the RotateRecovery recovery behavior.
       */
      void runBehavior();

      /**
       * @brief  Destructor for the rotate recovery behavior
       */
      ~RotateRecovery();

    private:
      costmap_2d::Costmap2DROS* global_costmap_, *local_costmap_;
      costmap_2d::Costmap2D costmap_;
      std::string name_;
      tf::TransformListener* tf_;
      bool initialized_;
      double frequency_;
      base_local_planner::CostmapModel* world_model_;
  };
};
#endif  