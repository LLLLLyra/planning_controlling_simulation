#ifndef CHASSIS_H_
#define CHASSIS_H_

#include "nav_msgs/Odometry.h"

/**
 * @brief Chassis simulator, bicycle model
 *
 */
class Chassis {
 public:
  Chassis() = default;
  /**
   * @brief Construct a new Chassis object
   *
   * @param odom initial odom
   */
  Chassis(nav_msgs::Odometry& odom);
  /**
   * @brief Construct a new Chassis object
   *
   * @param odom initial odom
   * @param wheel_base
   * @param steer_ratio
   */
  Chassis(nav_msgs::Odometry& odom, double wheel_base, double steer_ratio);
  /**
   * @brief Construct a new Chassis object
   *
   * @param wheel_base
   * @param steer_ratio
   */
  Chassis(double wheel_base, double steer_ratio);

  void set_wheel_base(double wheel_base) { wheel_base_ = wheel_base; }
  void set_steer_ratio(double steer_ratio) { steer_ratio_ = steer_ratio; }

  /**
   * @brief Update pose
   *
   * @param v speed m/s
   * @param steer steer wheel angle rad
   * @return new pose
   */
  nav_msgs::Odometry Update(double v, double steer);
  nav_msgs::Odometry get_odom() { return odom_; }

  std::string DebugString();

 private:
  nav_msgs::Odometry odom_;
  // time step
  double dt_ = 0.1;
  double wheel_base_ = 2.72;
  double steer_ratio_ = 17.2;
};

#endif  // CHASSIS_H_