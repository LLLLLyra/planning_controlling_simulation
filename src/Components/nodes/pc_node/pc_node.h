#ifndef PC_NODE_H_
#define PC_NODE_H_

#include "common/config/flags.h"
#include "common/proto/chassis.pb.h"
#include "common/proto/localization.pb.h"
#include "control/controller/lat_lon_controller.h"
#include "control/controller/simple_pid_lat_lon_controller.h"
#include "controlling_proc.h"
#include "display.h"
#include "nav_msgs/Odometry.h"
#include "planning_proc.h"
#include "ros/console.h"
#include "ros/ros.h"
#include "simulation_msg/CarControlMsg.h"
#include "simulation_msg/ObstacleService.h"

/**
 * @brief Planning and Control Main Node
 *
 */
class PCNode {
 public:
  PCNode(ros::NodeHandle* n);
  void Run();
  void localisation_call_back(const nav_msgs::OdometryPtr odometry);

 private:
  void Process();

 private:
  ros::NodeHandle n_;
  ros::Subscriber location_sub_;
  ros::Publisher cmd_pub_;
  ros::ServiceClient obstacles_map_service_;
  simulation_msg::ObstacleService obstacles_info_;

  Show show_;
  nav_msgs::Path reference_path_;
  nav_msgs::Path real_path_;
  nav_msgs::Path trajectory_;

  controller::LocalizationEstimate location_;
  canbus::Chassis chassis_;
  std::shared_ptr<DependencyInjector> injector_ =
      std::make_shared<DependencyInjector>();
  Controller* ctr_;

  std::queue<planning::ADCTrajectory> trajectory_queue_;
  ObstacleInfo obstacles_;
};

#endif