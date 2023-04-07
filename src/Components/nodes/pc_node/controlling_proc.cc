#include "controlling_proc.h"

#include "simulation_msg/CarControlMsg.h"

void ControlProc::Init(ros::Publisher* cmd_pub, Show* show) {
  cmd_pub_ = *cmd_pub;
  show_ = show;
}

void ControlProc::ControllerInit(Controller* ctr) { ctr_ = ctr; }

void ControlProc::ControlOnTrajectory(
    controller::LocalizationEstimate* location, canbus::Chassis* chassis,
    planning::ADCTrajectory* trajectory) {
  auto rate = ros::Rate(10);
  while (!ctr_->get_reached()) {
    Controlling(location, chassis, trajectory);
    rate.sleep();
  }
}

void ControlProc::Control(
    controller::LocalizationEstimate* location, canbus::Chassis* chassis,
    std::queue<planning::ADCTrajectory>* trajectory_queue) {
  while (!trajectory_queue->empty()) {
    planning::ADCTrajectory t = trajectory_queue->front();
    ControlOnTrajectory(location, chassis, &t);
    trajectory_queue->pop();
    ctr_->Reset();
  }
}

void ControlProc::Control(controller::LocalizationEstimate* location,
                          canbus::Chassis* chassis,
                          std::queue<planning::ADCTrajectory>* trajectory_queue,
                          std::shared_ptr<DependencyInjector>& injector) {
  while (!trajectory_queue->empty()) {
    planning::ADCTrajectory t = trajectory_queue->front();
    chassis->set_gear_location(t.gear());
    injector->vehicle_state()->Update(*location, *chassis);
    ControlOnTrajectory(location, chassis, &t);
    trajectory_queue->pop();
    ctr_->Reset();
  }
}

bool ControlProc::LatticePlanningSignal(planning::ADCTrajectory& trajectory) {
  auto target_point = ctr_->get_target_point();
  return (target_point.path_point().x() ==
          (trajectory.trajectory_point().end() - 1)->path_point().x()) &&
         (target_point.path_point().y() ==
          (trajectory.trajectory_point().end() - 1)->path_point().y());
}

void ControlProc::Controlling(controller::LocalizationEstimate* location,
                              canbus::Chassis* chassis,
                              planning::ADCTrajectory* trajectory) {
  controller::ControlCommand cmd;
  simulation_msg::CarControlMsg cmd_msg;
  ctr_->ComputeControlCommand(location, chassis, trajectory, &cmd);
  cmd_msg.gear = trajectory->gear();
  double speed = std::ceil(cmd.speed() * 3.6);
  cmd_msg.velocity = speed;
  if (cmd_msg.velocity == 0 && cmd.speed() != 0.0) {
    cmd_msg.velocity = speed > 0 ? 1 : -1;
  }
  cmd_msg.steering_angle = cmd.steering_target() / 100.0 *
                           veh_param_.max_steer_angle() / M_PI * 180.0;
  ROS_INFO_STREAM("v: " << cmd_msg.velocity
                        << ", steer: " << cmd_msg.steering_angle);
  cmd_pub_.publish(cmd_msg);
  show_->TargetPointToShow(ctr_->get_target_point());
  show_->RealPub();
}