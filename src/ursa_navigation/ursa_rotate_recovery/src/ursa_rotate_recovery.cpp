/*********************************************************************
* Rotate recovery behaviour for URSA drone
*
* Author: LAOSAAC
*********************************************************************/
#include <ursa_rotate_recovery/ursa_rotate_recovery.h>
#include <pluginlib/class_list_macros.h>

//register this planner as a RecoveryBehavior plugin
PLUGINLIB_DECLARE_CLASS(ursa_rotate_recovery, RotateRecovery, ursa_rotate_recovery::RotateRecovery, nav_core::RecoveryBehavior)

namespace ursa_rotate_recovery {

RotateRecovery::RotateRecovery(): global_costmap_(NULL), local_costmap_(NULL), 
  tf_(NULL), initialized_(false), world_model_(NULL) {} 

void RotateRecovery::initialize(std::string name, tf::TransformListener* tf,
    costmap_2d::Costmap2DROS* global_costmap, costmap_2d::Costmap2DROS* local_costmap){
  if(!initialized_){
    name_ = name;
    tf_ = tf;
    global_costmap_ = global_costmap;
    local_costmap_ = local_costmap;

    //get some parameters from the parameter server
    ros::NodeHandle private_nh("~/" + name_);

    //we'll check at 20Hz by default
    private_nh.param("frequency", frequency_, 20.0);


    world_model_ = new base_local_planner::CostmapModel(*local_costmap_->getCostmap());

    initialized_ = true;
  }
  else{
    ROS_ERROR("You should not call initialize twice on this object, doing nothing");
  }
}

RotateRecovery::~RotateRecovery(){
  delete world_model_;
}

void RotateRecovery::runBehavior(){
  if(!initialized_){
    ROS_ERROR("This object must be initialized before runBehavior is called");
    return;
  }

  if(global_costmap_ == NULL || local_costmap_ == NULL){
    ROS_ERROR("The costmaps passed to the RotateRecovery object cannot be NULL. Doing nothing.");
    return;
  }
  ROS_WARN("URSA UAV attempting to rotate to clear out space...");

  ros::Rate r(frequency_);
  ros::NodeHandle n;
  ros::Publisher target_pub = n.advertise<geometry_msgs::PoseStamped>("ursa_target", 1);

  tf::Stamped<tf::Pose> global_pose;
  local_costmap_->getRobotPose(global_pose);
  double start_angle = angles::normalize_angle(tf::getYaw(global_pose.getRotation()));
  double current_angle = start_angle;
  double angleToEnd;

  bool got_90 = false;

  tf::Stamped<tf::Pose> targetPose =
              tf::Stamped<tf::Pose>(tf::Pose(
                      tf::createQuaternionFromYaw(current_angle+M_PI),
                      tf::Point(global_pose.getOrigin().x(), global_pose.getOrigin().y(), 0.0)),
                      ros::Time::now(),
                      global_pose.frame_id_);
  geometry_msgs::PoseStamped target;
  tf::poseStampedTFToMsg(targetPose, target);
  target_pub.publish(target);

  while(n.ok()){
    local_costmap_->getRobotPose(global_pose);
    current_angle = angles::normalize_angle(tf::getYaw(global_pose.getRotation()));
    angleToEnd = fabs(angles::shortest_angular_distance(start_angle+M_PI,current_angle));
    if (angleToEnd < 0.2) return;
    r.sleep();
  }
}


}; // END NAMESPACE
