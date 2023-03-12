#ifndef _CRUISE_TRAJECTORY_H_
#define _CRUISE_TRAJECTORY_H_

#include "common/math/discrete_points_math.h"
#include "common/proto/vehicle_config.pb.h"
#include "discrete_points_reference_line_smoother.h"
#include "glog/logging.h"
#include "speed_generator.h"

using RFPS = std::vector<ReferencePoint>;

class CruiseTrajectory {
 public:
  CruiseTrajectory() = default;
  CruiseTrajectory(vehicle::VehicleParam vehicle_param)
      : vehicle_param_(vehicle_param){};
  virtual ~CruiseTrajectory() = default;

  // @brief Load path points
  // @param xy_points pairs of (x, y) coordinates
  // @param smoothing smooth the path or not
  // @param ref_points generated reference points
  // @return Whether to load path successfully
  bool LoadPathPoints(std::vector<std::pair<double, double>>& xy_points,
                      bool smoothing, RFPS* ref_points);

  // @brief Load path points from a file
  bool LoadPathPoints(const std::string file_name, bool smoothing,
                      RFPS* ref_points);

  std::vector<std::pair<double, double>> get_xy_points() { return xy_points_; }
  void set_xy_points(std::vector<std::pair<double, double>>& xy_points) {
    xy_points_ = xy_points;
  }

  RFPS get_ref_points() { return ref_points_; }

  KineticResult get_kinetic_result() { return kinetic_result_; }

  // @brief Get kinetics information of trajectory
  // @return Whether to generate trajectory successfully
  // @param ref_points reference points
  bool GenerateTrajectory(RFPS ref_points);
  bool GenerateTrajectory();

 private:
  bool LoadDataFromFile(const std::string file_name);

  // @brief drop the overlaps of a circled path
  // @param xy_points cooridinates of the path
  // @return success or not
  bool CutOverlap(std::vector<std::pair<double, double>>* xy_points);
  bool CutFromTail(std::vector<std::pair<double, double>>* xy_points);
  bool CutFromHead(std::vector<std::pair<double, double>>* xy_points);

  double GetDistanceSquared(std::pair<double, double>& a,
                            std::pair<double, double>& b);

 private:
  RFPS ref_points_;
  std::vector<std::pair<double, double>> xy_points_;
  KineticResult kinetic_result_;

  vehicle::VehicleParam vehicle_param_;
  planning::PiecewiseJerkSpeedOptimizerConfig piecewise_conf_;
};

#endif