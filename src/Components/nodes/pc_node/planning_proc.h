#ifndef PLANNING_PROC_H_
#define PLANNING_PROC_H_

#include "deciders.h"
#include "display.h"
#include "planning/common/pub_trajectory.h"
#include "planning/planner/lattice_planner.h"
#include "planning/planner/open_space_planner.h"

/**
 * @brief Planning Process
 *
 */
class PlanningProc {
 public:
  PlanningProc() = default;

  /**
   * @brief init display component
   *
   * @param show display component
   */
  void InitShow(Show* show);

  /**
   * @brief init lattice planner
   *
   * @param file_name reference path file
   */
  void LatticeInit(const std::string& file_name);
  /**
   * @brief init open space planner
   *
   * @param obstacles transfered obstacles
   */
  void OpenSpaceInit(ObstacleInfo& obstacles);

  /**
   * @brief lattice plan begins
   *
   * @param start_point
   * @param trajectory
   * @return success or not
   */
  bool LatticePlan(points::TrajectoryPoint& start_point,
                   planning::ADCTrajectory* trajectory);
  /**
   * @brief open space plan begins
   *
   * @param start
   * @param trajectories
   * @return success or not
   */
  bool OpenSpacePlan(std::array<double, 3>& start,
                     std::vector<planning::ADCTrajectory>* trajectories);
  /**
   * @brief cruising plan begins
   *
   * @param file_name recorded path[reference] file
   * @param trajectory
   * @return success or not
   */
  bool CruisePlan(const std::string& file_name,
                  planning::ADCTrajectory* trajectory);

 private:
  int cnt_ = 0;
  Show* show_ = nullptr;
  std::shared_ptr<LatticePlanner> lattice_;
  std::vector<std::vector<Vec2d>> obstacles_;
  std::vector<double> XYbounds_;
  std::array<double, 3> end_;
};

#endif  // PLANNING_PROC_H_