#ifndef CHASSIS_NODE_H_
#define CHASSIS_NODE_H_

#include "chassis/chassis.h"
#include "ros/console.h"
#include "ros/ros.h"
#include "simulation_msg/CarControlMsg.h"

/**
 * @brief Chassis Node
 *
 */
class ChassisNode {
 public:
  ChassisNode(ros::NodeHandle* n, Chassis& Chassis);
  void Run();

  void control_call_back(const simulation_msg::CarControlMsgConstPtr cmd);

 private:
  ros::NodeHandle n_;
  ros::Subscriber cmd_sub_;
  ros::Publisher location_pub_;
  Chassis chassis_;
};

#endif  // CHASSIS_NODE_H_