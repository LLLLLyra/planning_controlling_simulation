#ifndef _TRAJECTORY_STITCHER_H_
#define _TRAJECTORY_STITCHER_H_

#include <string>
#include <utility>
#include <vector>

#include "common/config/flags.h"
#include "common/proto/pnc_point.pb.h"
#include "common/proto/vehicle_state.pb.h"
#include "discretized_trajectory.h"

class PublishableTrajectory : public DiscretizedTrajectory {
 public:
  PublishableTrajectory() = default;
  PublishableTrajectory(const double header_time,
                        const DiscretizedTrajectory& discretized_trajectory)
      : DiscretizedTrajectory(discretized_trajectory),
        header_time_(header_time){};
  double header_time() const { return header_time_; };
  void set_header_time(double head_time) { header_time_ = head_time; };

 private:
  double header_time_ = 0.0;
};

class VehicleModel {
 public:
  VehicleModel() = delete;

  static vehicle_state::VehicleState Predict(
      const double predicted_time_horizon,
      const vehicle_state::VehicleState& cur_vehicle_state);

 private:
  static void RearCenteredKinematicBicycleModel(
      const double predicted_time_horizon,
      const vehicle_state::VehicleState& cur_vehicle_state,
      vehicle_state::VehicleState* predicted_vehicle_state);
};

class TrajectoryStitcher {
 public:
  TrajectoryStitcher() = delete;

  static void TransformLastPublishedTrajectory(
      const double x_diff, const double y_diff, const double theta_diff,
      PublishableTrajectory* prev_trajectory);

  static std::vector<points::TrajectoryPoint> ComputeStitchingTrajectory(
      const vehicle_state::VehicleState& vehicle_state,
      const double current_timestamp, const double planning_cycle_time,
      const size_t preserved_points_num, const bool replan_by_offset,
      const PublishableTrajectory* prev_trajectory, std::string* replan_reason);

  static std::vector<points::TrajectoryPoint> ComputeReinitStitchingTrajectory(
      const double planning_cycle_time,
      const vehicle_state::VehicleState& vehicle_state);

 private:
  static std::pair<double, double> ComputePositionProjection(
      const double x, const double y,
      const points::TrajectoryPoint& matched_trajectory_point);

  static points::TrajectoryPoint ComputeTrajectoryPointFromVehicleState(
      const double planning_cycle_time,
      const vehicle_state::VehicleState& vehicle_state);
};

#endif
