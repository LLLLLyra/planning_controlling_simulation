#include "trajectory_stitcher.h"

#include <algorithm>


#include "glog/logging.h"
#include "common/math/angle.h"
#include "common/math/quaternion.h"

void VehicleModel::RearCenteredKinematicBicycleModel(
    const double predicted_time_horizon, const vehicle_state::VehicleState& cur_vehicle_state,
    vehicle_state::VehicleState* predicted_vehicle_state) {
  // Kinematic bicycle model centered at rear axis center by Euler forward
  // discretization
  // Assume constant control command and constant z axis position
  CHECK_GT(predicted_time_horizon, 0.0);
  double dt = cur_vehicle_state.dt();
  double cur_x = cur_vehicle_state.x();
  double cur_y = cur_vehicle_state.y();
  double cur_z = cur_vehicle_state.z();
  double cur_phi = cur_vehicle_state.heading();
  double cur_v = cur_vehicle_state.linear_velocity();
  double cur_a = cur_vehicle_state.linear_acceleration();
  double next_x = cur_x;
  double next_y = cur_y;
  double next_phi = cur_phi;
  double next_v = cur_v;
  if (dt >= predicted_time_horizon) {
    dt = predicted_time_horizon;
  }

  double countdown_time = predicted_time_horizon;
  bool finish_flag = false;
  static constexpr double kepsilon = 1e-8;
  while (countdown_time > kepsilon && !finish_flag) {
    countdown_time -= dt;
    if (countdown_time < kepsilon) {
      dt = countdown_time + dt;
      finish_flag = true;
    }
    double intermidiate_phi =
        cur_phi + 0.5 * dt * cur_v * cur_vehicle_state.kappa();
    next_phi =
        cur_phi + dt * (cur_v + 0.5 * dt * cur_a) * cur_vehicle_state.kappa();
    next_x =
        cur_x + dt * (cur_v + 0.5 * dt * cur_a) * std::cos(intermidiate_phi);
    next_y =
        cur_y + dt * (cur_v + 0.5 * dt * cur_a) * std::sin(intermidiate_phi);

    next_v = cur_v + dt * cur_a;
    cur_x = next_x;
    cur_y = next_y;
    cur_phi = next_phi;
    cur_v = next_v;
  }

  predicted_vehicle_state->set_x(next_x);
  predicted_vehicle_state->set_y(next_y);
  predicted_vehicle_state->set_z(cur_z);
  predicted_vehicle_state->set_heading(next_phi);
  predicted_vehicle_state->set_kappa(cur_vehicle_state.kappa());
  predicted_vehicle_state->set_linear_velocity(next_v);
  predicted_vehicle_state->set_linear_acceleration(
      cur_vehicle_state.linear_acceleration());
}

vehicle_state::VehicleState VehicleModel::Predict(const double predicted_time_horizon,
                                   const vehicle_state::VehicleState& cur_vehicle_state) {
  vehicle_state::VehicleState predicted_vehicle_state;
  RearCenteredKinematicBicycleModel(predicted_time_horizon, cur_vehicle_state,
                                      &predicted_vehicle_state);

  return predicted_vehicle_state;
}



points::TrajectoryPoint TrajectoryStitcher::ComputeTrajectoryPointFromVehicleState(
    const double planning_cycle_time, const vehicle_state::VehicleState& vehicle_state) {
  points::TrajectoryPoint point;
  point.mutable_path_point()->set_s(0.0);
  point.mutable_path_point()->set_x(vehicle_state.x());
  point.mutable_path_point()->set_y(vehicle_state.y());
  point.mutable_path_point()->set_z(vehicle_state.z());
  point.mutable_path_point()->set_theta(vehicle_state.heading());
  point.mutable_path_point()->set_kappa(vehicle_state.kappa());
  point.set_v(vehicle_state.linear_velocity());
  point.set_a(vehicle_state.linear_acceleration());
  point.set_relative_time(planning_cycle_time);
  return point;
}

std::vector<points::TrajectoryPoint>
TrajectoryStitcher::ComputeReinitStitchingTrajectory(
    const double planning_cycle_time, const vehicle_state::VehicleState& vehicle_state) {
  points::TrajectoryPoint reinit_point;
  static constexpr double kEpsilon_v = 0.1;
  static constexpr double kEpsilon_a = 0.4;
  // TODO(Jinyun/Yu): adjust kEpsilon if corrected IMU acceleration provided
  if (std::abs(vehicle_state.linear_velocity()) < kEpsilon_v &&
      std::abs(vehicle_state.linear_acceleration()) < kEpsilon_a) {
    reinit_point = ComputeTrajectoryPointFromVehicleState(planning_cycle_time,
                                                          vehicle_state);
  } else {
    vehicle_state::VehicleState predicted_vehicle_state;
    predicted_vehicle_state =
        VehicleModel::Predict(planning_cycle_time, vehicle_state);
    reinit_point = ComputeTrajectoryPointFromVehicleState(
        planning_cycle_time, predicted_vehicle_state);
  }

  return std::vector<points::TrajectoryPoint>(1, reinit_point);
}

// only used in navigation mode
void TrajectoryStitcher::TransformLastPublishedTrajectory(
    const double x_diff, const double y_diff, const double theta_diff,
    PublishableTrajectory* prev_trajectory) {
  if (!prev_trajectory) {
    return;
  }

  // R^-1
  double cos_theta = std::cos(theta_diff);
  double sin_theta = -std::sin(theta_diff);

  // -R^-1 * t
  auto tx = -(cos_theta * x_diff - sin_theta * y_diff);
  auto ty = -(sin_theta * x_diff + cos_theta * y_diff);

  std::for_each(prev_trajectory->begin(), prev_trajectory->end(),
                [&cos_theta, &sin_theta, &tx, &ty,
                 &theta_diff](points::TrajectoryPoint& p) {
                  auto x = p.path_point().x();
                  auto y = p.path_point().y();
                  auto theta = p.path_point().theta();

                  auto x_new = cos_theta * x - sin_theta * y + tx;
                  auto y_new = sin_theta * x + cos_theta * y + ty;
                  auto theta_new =
                      NormalizeAngle(theta - theta_diff);

                  p.mutable_path_point()->set_x(x_new);
                  p.mutable_path_point()->set_y(y_new);
                  p.mutable_path_point()->set_theta(theta_new);
                });
}

/* Planning from current vehicle state if:
   1. the auto-driving mode is off
   (or) 2. we don't have the trajectory from last planning cycle
   (or) 3. the position deviation from actual and target is too high
*/
std::vector<points::TrajectoryPoint> TrajectoryStitcher::ComputeStitchingTrajectory(
    const vehicle_state::VehicleState& vehicle_state, const double current_timestamp,
    const double planning_cycle_time, const size_t preserved_points_num,
    const bool replan_by_offset, const PublishableTrajectory* prev_trajectory,
    std::string* replan_reason) {
  if (!FLAGS_enable_trajectory_stitcher) {
    *replan_reason = "stitch is disabled by gflag.";
    return ComputeReinitStitchingTrajectory(planning_cycle_time, vehicle_state);
  }
  if (!prev_trajectory || prev_trajectory->empty()) {
    *replan_reason = "replan for no previous trajectory.";
    return ComputeReinitStitchingTrajectory(planning_cycle_time, vehicle_state);
  }

  size_t prev_trajectory_size = prev_trajectory->NumOfPoints();

  if (prev_trajectory_size == 0) {
    LOG(INFO) << "Projected trajectory at time [" << prev_trajectory->header_time()
           << "] size is zero! Previous planning not exist or failed. Use "
              "origin car status instead.";
    *replan_reason = "replan for empty previous trajectory.";
    return ComputeReinitStitchingTrajectory(planning_cycle_time, vehicle_state);
  }

  const double veh_rel_time =
      current_timestamp - prev_trajectory->header_time();

  size_t time_matched_index =
      prev_trajectory->QueryLowerBoundPoint(veh_rel_time);

  if (time_matched_index == 0 &&
      veh_rel_time < prev_trajectory->StartPoint().relative_time()) {
    LOG(INFO) << "current time smaller than the previous trajectory's first time";
    *replan_reason =
        "replan for current time smaller than the previous trajectory's first "
        "time.";
    return ComputeReinitStitchingTrajectory(planning_cycle_time, vehicle_state);
  }
  if (time_matched_index + 1 >= prev_trajectory_size) {
    LOG(INFO) << "current time beyond the previous trajectory's last time";
    *replan_reason =
        "replan for current time beyond the previous trajectory's last time";
    return ComputeReinitStitchingTrajectory(planning_cycle_time, vehicle_state);
  }

  auto time_matched_point = prev_trajectory->TrajectoryPointAt(
      static_cast<uint32_t>(time_matched_index));

  if (!time_matched_point.has_path_point()) {
    *replan_reason = "replan for previous trajectory missed path point";
    return ComputeReinitStitchingTrajectory(planning_cycle_time, vehicle_state);
  }

  size_t position_matched_index = prev_trajectory->QueryNearestPointWithBuffer(
      {vehicle_state.x(), vehicle_state.y()}, 1.0e-6);

  auto frenet_sd = ComputePositionProjection(
      vehicle_state.x(), vehicle_state.y(),
      prev_trajectory->TrajectoryPointAt(
          static_cast<uint32_t>(position_matched_index)));

  if (replan_by_offset) {
    auto lon_diff = time_matched_point.path_point().s() - frenet_sd.first;
    auto lat_diff = frenet_sd.second;

    LOG(INFO) << "Control lateral diff: " << lat_diff
           << ", longitudinal diff: " << lon_diff;

    if (std::fabs(lat_diff) > FLAGS_replan_lateral_distance_threshold) {
      const std::string msg = 
          "the distance between matched point and actual position is too "
          "large. Replan is triggered. lat_diff = " + 
          std::to_string(lat_diff);
      LOG(ERROR) << msg;
      *replan_reason = msg;
      return ComputeReinitStitchingTrajectory(planning_cycle_time,
                                              vehicle_state);
    }

    if (std::fabs(lon_diff) > FLAGS_replan_longitudinal_distance_threshold) {
      const std::string msg = 
          "the distance between matched point and actual position is too "
          "large. Replan is triggered. lon_diff = " + 
          std::to_string(lon_diff);
      LOG(ERROR) << msg;
      *replan_reason = msg;
      return ComputeReinitStitchingTrajectory(planning_cycle_time,
                                              vehicle_state);
    }
  } else {
    LOG(INFO) << "replan according to certain amount of lat and lon offset is "
              "disabled";
  }

  double forward_rel_time = veh_rel_time + planning_cycle_time;

  size_t forward_time_index =
      prev_trajectory->QueryLowerBoundPoint(forward_rel_time);

  LOG(INFO) << "Position matched index:\t" << position_matched_index;
  LOG(INFO) << "Time matched index:\t" << time_matched_index;

  auto matched_index = std::min(time_matched_index, position_matched_index);

  std::vector<points::TrajectoryPoint> stitching_trajectory(
      prev_trajectory->begin() +
          std::max(0, static_cast<int>(matched_index - preserved_points_num)),
      prev_trajectory->begin() + forward_time_index + 1);
  LOG(INFO) << "stitching_trajectory size: " << stitching_trajectory.size();

  const double zero_s = stitching_trajectory.back().path_point().s();
  for (auto& tp : stitching_trajectory) {
    if (!tp.has_path_point()) {
      *replan_reason = "replan for previous trajectory missed path point";
      return ComputeReinitStitchingTrajectory(planning_cycle_time,
                                              vehicle_state);
    }
    tp.set_relative_time(tp.relative_time() + prev_trajectory->header_time() -
                         current_timestamp);
    tp.mutable_path_point()->set_s(tp.path_point().s() - zero_s);
  }
  return stitching_trajectory;
}

std::pair<double, double> TrajectoryStitcher::ComputePositionProjection(
    const double x, const double y, const points::TrajectoryPoint& p) {
  Vec2d v(x - p.path_point().x(), y - p.path_point().y());
  Vec2d n(std::cos(p.path_point().theta()), std::sin(p.path_point().theta()));

  std::pair<double, double> frenet_sd;
  frenet_sd.first = v.InnerProd(n) + p.path_point().s();
  frenet_sd.second = v.CrossProd(n);
  return frenet_sd;
}

