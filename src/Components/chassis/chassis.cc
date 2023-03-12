#include "chassis.h"

#include "common/math/quaternion.h"

Chassis::Chassis(nav_msgs::Odometry& odom) : odom_(odom) {}

Chassis::Chassis(nav_msgs::Odometry& odom, double wheel_base,
                 double steer_ratio)
    : odom_(odom), wheel_base_(wheel_base), steer_ratio_(steer_ratio) {}

Chassis::Chassis(double wheel_base, double steer_ratio)
    : wheel_base_(wheel_base), steer_ratio_(steer_ratio) {}

nav_msgs::Odometry Chassis::Update(double v, double steer) {
  double qw = odom_.pose.pose.orientation.w;
  double qx = odom_.pose.pose.orientation.x;
  double qy = odom_.pose.pose.orientation.y;
  double qz = odom_.pose.pose.orientation.z;

  double heading = QuaternionToHeading(qw, qx, qy, qz);
  heading = NormalizeAngle(heading + v * std::tan(steer / steer_ratio_) /
                                         wheel_base_ * dt_);
  auto ori = HeadingToQuaternion(heading);
  odom_.pose.pose.orientation.w = ori.w();
  odom_.pose.pose.orientation.x = ori.x();
  odom_.pose.pose.orientation.y = ori.y();
  odom_.pose.pose.orientation.z = ori.z();
  odom_.pose.pose.position.x += v * std::cos(heading) * dt_;
  odom_.pose.pose.position.y += v * std::sin(heading) * dt_;

  return odom_;
}

std::string Chassis::DebugString() {
  double qw = odom_.pose.pose.orientation.w;
  double qx = odom_.pose.pose.orientation.x;
  double qy = odom_.pose.pose.orientation.y;
  double qz = odom_.pose.pose.orientation.z;
  double x = odom_.pose.pose.position.x;
  double y = odom_.pose.pose.position.y;

  double heading = QuaternionToHeading(qw, qx, qy, qz);
  char buffer[100];
  memset(buffer, 0, sizeof(buffer));
  sprintf(buffer, "Current Vehicle State: x = %.6f, y = %.6f, theta = %.6f \n",
          x, y, heading);
  return std::string(buffer);
}