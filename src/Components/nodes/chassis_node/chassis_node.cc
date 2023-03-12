#include "chassis_node.h"

#include "common/math/quaternion.h"

ChassisNode::ChassisNode(ros::NodeHandle* n, Chassis& Chassis)
    : n_(*n), chassis_(Chassis) {
  location_pub_ = n_.advertise<nav_msgs::Odometry>("/location_odom", 100);
}

void ChassisNode::Run() {
  auto rate = ros::Rate(10);
  location_pub_.publish(chassis_.Update(0, 0));
  while (ros::ok()) {
    location_pub_.publish(chassis_.get_odom());
    cmd_sub_ = n_.subscribe("/car_control", 1000,
                            &ChassisNode::control_call_back, this);
    rate.sleep();
    ros::spinOnce();
  }
}

void ChassisNode::control_call_back(
    const simulation_msg::CarControlMsgConstPtr cmd) {
  double v = static_cast<double>(cmd->velocity) / 3.6;
  double steer = M_PI * static_cast<double>(cmd->steering_angle) / 180.0;
  chassis_.Update(v, steer);
  ROS_INFO_STREAM(chassis_.DebugString());
}

int main(int args, char* argv[]) {
  ros::init(args, argv, "Chassis");
  ros::NodeHandle n;
  nav_msgs::Odometry odom;

  // In this example, we set the initial pose of the ego to be [0, 0, -pi/2]
  auto ori = HeadingToQuaternion(-M_PI_2);
  odom.pose.pose.orientation.w = ori.w();
  odom.pose.pose.orientation.x = ori.x();
  odom.pose.pose.orientation.y = ori.y();
  odom.pose.pose.orientation.z = ori.z();
  Chassis chassis(odom);

  ChassisNode node(&n, chassis);
  node.Run();
}
