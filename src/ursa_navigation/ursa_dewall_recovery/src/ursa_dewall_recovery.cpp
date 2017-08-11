/*********************************************************************
* De-wall recovery behaviour for URSA drone
*
* Author: LAOSAAC
*********************************************************************/
#include <ursa_dewall_recovery/ursa_dewall_recovery.h>
#include <pluginlib/class_list_macros.h>

//register this planner as a RecoveryBehavior plugin
PLUGINLIB_DECLARE_CLASS(ursa_dewall_recovery, DeWall, ursa_dewall_recovery::DeWall, nav_core::RecoveryBehavior)

namespace ursa_dewall_recovery {

DeWall::DeWall(): global_costmap_(NULL), local_costmap_(NULL), 
  tf_(NULL), initialized_(false), world_model_(NULL) {} 

void DeWall::initialize(std::string name, tf::TransformListener* tf,
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

    // alpha for gradient descent parameter
    private_nh.param("alpha", alpha_, 0.8);


    //world_model_ = new base_local_planner::CostmapModel(*local_costmap_->getCostmap());

    initialized_ = true;
  }
  else{
    ROS_ERROR("You should not call initialize twice on this object, doing nothing");
  }
}

DeWall::~DeWall(){
  //delete world_model_;
}

void DeWall::runBehavior(){
  if(!initialized_){
    ROS_ERROR("This object must be initialized before runBehavior is called");
    return;
  }

  if(global_costmap_ == NULL || local_costmap_ == NULL){
    ROS_ERROR("The costmaps passed to the DeWall object cannot be NULL. Doing nothing.");
    return;
  }
  ROS_WARN("URSA UAV attempting to move away from any walls...");

  ros::Rate r(frequency_);
  ros::NodeHandle n;
  ros::Publisher target_pub = n.advertise<geometry_msgs::PoseStamped>("ursa_target", 1);

  costmap_2d::Costmap2D* costmap = local_costmap_->getCostmap();
  double xw, yw, dx, dy;
  unsigned int xm, ym;
  int div = 1;

  tf::Stamped<tf::Pose> global_pose;
  geometry_msgs::PoseStamped target;
  double start_angle = angles::normalize_angle(tf::getYaw(global_pose.getRotation()));
  tf::Stamped<tf::Pose> targetPose;

  while(n.ok()){
    r.sleep();
    //boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*(costmap->getMutex()));
    local_costmap_->getRobotPose(global_pose);
    costmap = local_costmap_->getCostmap();
    costmap->worldToMap(global_pose.getOrigin().x(), global_pose.getOrigin().x(), xm, ym);
    dx = costmap->getCost(xm-1, ym) - costmap->getCost(xm+1, ym);
    dy = costmap->getCost(xm, ym-1) - costmap->getCost(xm, ym+1);
    if (fabs(dx*alpha_)>=costmap->getSizeInCellsX()/2){
      if (dx>0) dx=1.0f*costmap->getSizeInCellsX()/2/alpha_-3;
      else dx=-1.0f*costmap->getSizeInCellsX()/2/alpha_+3;
    }
    if (fabs(dy*alpha_)>=costmap->getSizeInCellsY()/2){
      if (dy>0) dy=1.0f*costmap->getSizeInCellsY()/2/alpha_-3;
      else dy=-1.0f*costmap->getSizeInCellsY()/2/alpha_+3;
    }
    costmap->mapToWorld(xm+dx*alpha_, ym+dy*alpha_, xw, yw);
    targetPose = tf::Stamped<tf::Pose>(tf::Pose(
                      tf::createQuaternionFromYaw(start_angle),
                      tf::Point(xw, yw, 0.0)),
                      ros::Time::now(),
                      global_pose.frame_id_);
    tf::poseStampedTFToMsg(targetPose, target);
    target_pub.publish(target);
    if (costmap->getCost(xm, ym)==0){
      ROS_INFO("Successfully navigated away from walls.");
      return;
    }
  }
}


}; // END NAMESPACE
