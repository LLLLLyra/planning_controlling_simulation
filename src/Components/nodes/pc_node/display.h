#ifndef DISPLAY_H_
#define DISPLAY_H_

#include "common/math/vec2d.h"
#include "common/proto/planning.pb.h"
#include "common/proto/vehicle_config.pb.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"
#include "planning/common/reference_line.h"
#include "ros/console.h"
#include "ros/ros.h"
#include "visualization_msgs/Marker.h"

class Show {
 public:
  Show() = default;
  Show(ros::NodeHandle* n, nav_msgs::Path* reference_path,
       nav_msgs::Path* trajectory, nav_msgs::Path* real_path);

  void RealPub();

  void Init(ros::NodeHandle* n, nav_msgs::Path* reference_path,
            nav_msgs::Path* trajectory, nav_msgs::Path* real_path);

  void TrajectoryToShow(const planning::ADCTrajectory& trajectory);
  void ReferencePathToShow(const ReferenceLine& ref_line);
  void RealPathToShow(const nav_msgs::OdometryPtr& odometry);
  void TargetPointToShow(const points::TrajectoryPoint& point);
  void ObstaclePointToShow(const std::vector<std::vector<Vec2d>>& obstacles);

 private:
  void MarkerInit();

 private:
  ros::NodeHandle* n_ = nullptr;
  ros::Publisher reference_path_pub_;
  ros::Publisher trajectory_pub_;
  ros::Publisher real_time_path_pub_;
  ros::Publisher target_point_pub_;
  ros::Publisher obstacles_pub_;

  nav_msgs::Path* reference_path_ = nullptr;
  nav_msgs::Path* trajectory_ = nullptr;
  nav_msgs::Path* real_path_ = nullptr;
  nav_msgs::Path obstacles_;
  visualization_msgs::Marker marker_;
};

#endif  // DISPLAY_H_