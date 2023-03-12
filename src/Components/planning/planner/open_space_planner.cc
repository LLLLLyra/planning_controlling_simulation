#include "open_space_planner.h"

#include "planning/common/pub_trajectory.h"

bool OpenSpacePlanner::Plan(
    double sx, double sy, double sphi, double ex, double ey, double ephi,
    std::vector<double> XYbounds, std::vector<std::vector<Vec2d>> obstacles,
    std::vector<planning::ADCTrajectory>* trajectories) {
  planning::PlannerOpenSpaceConfig const config =
      planning::PlannerOpenSpaceConfig();
  HybridAStar ha(config);
  HybridAStarResult combined;
  bool planned =
      ha.Plan(sx, sy, sphi, ex, ey, ephi, XYbounds, obstacles, &combined);
  if (!planned) return false;
  std::vector<HybridAStarResult> partitions;
  if (!ha.TrajectoryPartition(combined, &partitions)) {
    return false;
  }

  auto begin = Timer::Now();
  for (auto& par : partitions) {
    planning::ADCTrajectory traj;
    if (!FLAGS_enable_IAPS ||
        par.accumulated_s.back() < FLAGS_min_trajectory_distance) {
      Planning::PublishTrajectory<HybridAStarResult>(&traj, par);
    } else {
      Eigen::MatrixXd xWS;
      Eigen::MatrixXd uWS;
      Eigen::MatrixXd state_result_ds;
      Eigen::MatrixXd control_result_ds;
      Eigen::MatrixXd time_result_ds;

      LoadHybridAStarResultInEigen(&par, &xWS, &uWS);

      const double init_steer = 0;
      const double init_a = 0;
      Eigen::MatrixXd last_time_u(2, 1);
      last_time_u << init_steer, init_a;
      const double init_v = 0;

      planning::PlannerOpenSpaceConfig confg;
      IterativeAnchoringSmoother iterative_anchoring_smoother(confg);

      DiscretizedTrajectory smoothed_trajectory;

      if (!iterative_anchoring_smoother.Smooth(xWS, last_time_u(1, 0), init_v,
                                               obstacles,
                                               &smoothed_trajectory)) {
        return false;
      }
      LoadResult(smoothed_trajectory, &state_result_ds, &control_result_ds,
                 &time_result_ds);
      LoadTrajectory(state_result_ds, control_result_ds, time_result_ds, &traj);
    }
    trajectories->emplace_back(traj);
  }
  double time = (Timer::Now() - begin) * 1000;
  LOG(INFO) << "It costs " << time << "ms to pub trajectories";
  return true;
}

void OpenSpacePlanner::LoadHybridAStarResultInEigen(HybridAStarResult* result,
                                                    Eigen::MatrixXd* xWS,
                                                    Eigen::MatrixXd* uWS) {
  size_t horizon = result->x.size() - 1;
  xWS->resize(4, horizon + 1);
  uWS->resize(2, horizon);
  Eigen::VectorXd x = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(
      result->x.data(), horizon + 1);
  Eigen::VectorXd y = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(
      result->y.data(), horizon + 1);
  Eigen::VectorXd phi = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(
      result->phi.data(), horizon + 1);
  Eigen::VectorXd v = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(
      result->v.data(), horizon + 1);
  Eigen::VectorXd steer = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(
      result->steer.data(), horizon);
  Eigen::VectorXd a =
      Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(result->a.data(), horizon);
  xWS->row(0) = std::move(x);
  xWS->row(1) = std::move(y);
  xWS->row(2) = std::move(phi);
  xWS->row(3) = std::move(v);
  uWS->row(0) = std::move(steer);
  uWS->row(1) = std::move(a);
}

void OpenSpacePlanner::LoadResult(
    const DiscretizedTrajectory& discretized_trajectory,
    Eigen::MatrixXd* state_result_dc, Eigen::MatrixXd* control_result_dc,
    Eigen::MatrixXd* time_result_dc) {
  const size_t points_size = discretized_trajectory.size();
  CHECK_GT(points_size, 1U);
  *state_result_dc = Eigen::MatrixXd::Zero(6, points_size);
  *control_result_dc = Eigen::MatrixXd::Zero(2, points_size - 1);
  *time_result_dc = Eigen::MatrixXd::Zero(1, points_size - 1);

  auto& state_result = *state_result_dc;
  for (size_t i = 0; i < points_size; ++i) {
    state_result(0, i) = discretized_trajectory[i].path_point().x();
    state_result(1, i) = discretized_trajectory[i].path_point().y();
    state_result(2, i) = discretized_trajectory[i].path_point().theta();
    state_result(3, i) = discretized_trajectory[i].v();
    state_result(4, i) = discretized_trajectory[i].path_point().kappa();
    state_result(5, i) = discretized_trajectory[i].path_point().s();
  }

  auto& control_result = *control_result_dc;
  auto& time_result = *time_result_dc;
  const double wheel_base = vehicle::VehicleParam().wheel_base();
  for (size_t i = 0; i + 1 < points_size; ++i) {
    control_result(0, i) =
        std::atan(discretized_trajectory[i].path_point().kappa() * wheel_base);
    control_result(1, i) = discretized_trajectory[i].a();
    time_result(0, i) = discretized_trajectory[i + 1].relative_time() -
                        discretized_trajectory[i].relative_time();
  }
}

void OpenSpacePlanner::LoadTrajectory(const Eigen::MatrixXd& state_result,
                                      const Eigen::MatrixXd& control_result,
                                      const Eigen::MatrixXd& time_result,
                                      planning::ADCTrajectory* trajectoy) {
  trajectoy->Clear();
  size_t counter = 0;
  size_t states_size = state_result.cols();
  size_t times_size = time_result.cols();
  size_t controls_size = control_result.cols();
  CHECK_EQ(states_size, times_size + 1);
  CHECK_EQ(states_size, controls_size + 1);
  double relative_time = 0.0;
  Vec2d last_path_point(state_result(0, 0), state_result(1, 0));
  for (size_t i = 0; i < states_size; ++i) {
    auto point = trajectoy->add_trajectory_point();
    point->mutable_path_point()->set_x(state_result(0, i));
    point->mutable_path_point()->set_y(state_result(1, i));
    point->mutable_path_point()->set_theta(state_result(2, i));
    point->set_v(state_result(3, i));
    point->mutable_path_point()->set_kappa(state_result(4, i));
    point->mutable_path_point()->set_s(state_result(5, i));
    counter += state_result(3, i) < 0;
    Vec2d cur_path_point(state_result(0, i), state_result(1, i));
    if (i == controls_size) {
      point->set_steer(0.0);
      point->set_a(0.0);
    } else {
      point->set_steer(control_result(0, i));
      point->set_a(control_result(1, i));
    }

    if (i == 0) {
      point->set_relative_time(relative_time);
    } else {
      relative_time += time_result(0, i - 1);
      point->set_relative_time(relative_time);
    }
    last_path_point = cur_path_point;
  }
  trajectoy->set_gear(counter > (states_size >> 1)
                          ? canbus::Chassis::GEAR_REVERSE
                          : canbus::Chassis::GEAR_DRIVE);
}