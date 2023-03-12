#ifndef CONTROLLING_PROC_H_
#define CONTROLLING_PROC_H_

#include <queue>

#include "common/proto/vehicle_config.pb.h"
#include "control/controller/controller.h"
#include "display.h"
#include "ros/ros.h"

/**
 * @brief Control Process
 *
 */
class ControlProc {
 public:
  ControlProc() = default;

  /**
   * @brief init command publisher and display component
   *
   * @param cmd_pub command publisher
   * @param show display component
   */
  void Init(ros::Publisher* cmd_pub, Show* show);
  /**
   * @brief init a controller
   *
   * @param ctr controller
   */
  void ControllerInit(Controller* ctr);

  /**
   * @brief control process for cruising
   *
   * @param location
   * @param chassis
   * @param trajectory_queue
   */
  void Control(controller::LocalizationEstimate* location,
               canbus::Chassis* chassis,
               std::queue<planning::ADCTrajectory>* trajectory_queue);

  /**
   * @brief control process for parking
   *
   * @param location
   * @param chassis
   * @param trajectory_queue
   * @param injector
   */
  void Control(controller::LocalizationEstimate* location,
               canbus::Chassis* chassis,
               std::queue<planning::ADCTrajectory>* trajectory_queue,
               std::shared_ptr<DependencyInjector>& injector);

  /**
   * @brief single step controlling
   *
   * @param location
   * @param chassis
   * @param trajectory
   */
  void Controlling(controller::LocalizationEstimate* location,
                   canbus::Chassis* chassis,
                   planning::ADCTrajectory* trajectory);

  /**
   * @brief check to call lattice planner
   *
   * @param trajectory trajectory of last planning frame
   * @return whether to call lattice planner or not
   */
  bool LatticePlanningSignal(planning::ADCTrajectory& trajectory);

 private:
  void ControlOnTrajectory(controller::LocalizationEstimate* location,
                           canbus::Chassis* chassis,
                           planning::ADCTrajectory* trajectory);

 private:
  ros::Publisher cmd_pub_;
  Show* show_;

  Controller* ctr_ = nullptr;
  vehicle::VehicleParam veh_param_;
};

#endif  // CONTROLLING_PROC_H_