#include "display.h"

Show::Show(ros::NodeHandle* n, nav_msgs::Path* reference_path,
           nav_msgs::Path* trajectory, nav_msgs::Path* real_path) {
  Init(n, reference_path, trajectory, real_path);
}

void Show::Init(ros::NodeHandle* n, nav_msgs::Path* reference_path,
                nav_msgs::Path* trajectory, nav_msgs::Path* real_path) {
  CHECK_NOTNULL(n);
  CHECK_NOTNULL(reference_path);
  CHECK_NOTNULL(trajectory);
  CHECK_NOTNULL(real_path);
  n_ = n;
  trajectory_ = trajectory;
  reference_path_ = reference_path;
  real_path_ = real_path;
  reference_path_pub_ = n_->advertise<nav_msgs::Path>("/reference_path", 1000);
  trajectory_pub_ = n_->advertise<nav_msgs::Path>("/trajectory", 1000);
  real_time_path_pub_ = n_->advertise<nav_msgs::Path>("/real_path", 1000);
  target_point_pub_ =
      n_->advertise<visualization_msgs::Marker>("/target_point", 1000);
  obstacles_pub_ = n_->advertise<nav_msgs::Path>("/obstacles", 1000);

  MarkerInit();
}

void Show::MarkerInit() {
  marker_.header.frame_id = "gps";
  marker_.ns = "control";
  marker_.id = 0;
  marker_.type = visualization_msgs::Marker::SPHERE;
  marker_.action = visualization_msgs::Marker::ADD;
  marker_.pose.position.z = 0;
  marker_.pose.orientation.x = 0.0;
  marker_.pose.orientation.y = 0.0;
  marker_.pose.orientation.z = 0.0;
  marker_.pose.orientation.w = 1.0;
  marker_.scale.x = 5;
  marker_.scale.y = 0.5;
  marker_.scale.z = 0.5;
  marker_.color.a = 1.0;  // Don't forget to set the alpha!
  marker_.color.r = 0.0;
  marker_.color.g = 1.0;
  marker_.color.b = 0.0;
  // only if using a MESH_RESOURCE marker type:
  marker_.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";
}

void Show::TrajectoryToShow(const planning::ADCTrajectory& trajectory) {
  trajectory_->header.frame_id = "gps";
  std::for_each(trajectory.trajectory_point().begin(),
                trajectory.trajectory_point().end(),
                [&](const points::TrajectoryPoint& point) -> void {
                  geometry_msgs::PoseStamped p;
                  p.header.frame_id = "gps";
                  p.pose.position.x = point.path_point().x();
                  p.pose.position.y = point.path_point().y();
                  trajectory_->poses.push_back(p);
                });
}

void Show::ReferencePathToShow(const ReferenceLine& ref_line) {
  reference_path_->header.frame_id = "gps";
  std::for_each(ref_line.reference_points().begin(),
                ref_line.reference_points().end(),
                [&](const ReferencePoint& point) -> void {
                  geometry_msgs::PoseStamped p;
                  p.header.frame_id = "gps";
                  p.pose.position.x = point.x();
                  p.pose.position.y = point.y();
                  reference_path_->poses.push_back(p);
                });
}

void Show::RealPathToShow(const nav_msgs::OdometryPtr& odometry) {
  geometry_msgs::PoseStamped pose;
  pose.header = odometry->header;
  pose.pose = odometry->pose.pose;
  real_path_->header.frame_id = "gps";
  pose.header.frame_id = "gps";
  real_path_->poses.push_back(pose);
}

void Show::TargetPointToShow(const points::TrajectoryPoint& point) {
  marker_.header.stamp = ros::Time();
  marker_.pose.position.x = point.path_point().x();
  marker_.pose.position.y = point.path_point().y();
}

void Show::ObstaclePointToShow(
    const std::vector<std::vector<Vec2d>>& obstacles) {
  obstacles_.header.frame_id = "gps";
  for (const auto& obstacle : obstacles) {
    for (const auto& vec : obstacle) {
      geometry_msgs::PoseStamped p;
      p.header.frame_id = "gps";
      p.pose.position.x = vec.x();
      p.pose.position.y = vec.y();
      obstacles_.poses.push_back(p);
    }
  }
}

void Show::RealPub() {
  const auto reference_path = *reference_path;
  const auto trajectory = *trajectory_;
  const auto real_path = *real_path_;
  const auto marker = marker_;
  const auto obstacles = obstacles_;
  reference_path_pub_.publish(reference_path);
  trajectory_pub_.publish(trajectory);
  real_time_path_pub_.publish(real_path);
  target_point_pub_.publish(marker);
  obstacles_pub_.publish(obstacles);
}
