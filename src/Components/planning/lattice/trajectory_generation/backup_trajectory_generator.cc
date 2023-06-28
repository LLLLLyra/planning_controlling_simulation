#include "backup_trajectory_generator.h"

#include "trajectory_combiner.h"

using State = std::array<double, 3>;

BackupTrajectoryGenerator::BackupTrajectoryGenerator(
    const State& init_s, const State& init_d, const double init_relative_time,
    const std::shared_ptr<CollisionChecker>& ptr_collision_checker,
    const Trajectory1dGenerator* trajectory1d_generator)
    : init_relative_time_(init_relative_time),
      ptr_collision_checker_(ptr_collision_checker),
      ptr_trajectory1d_generator_(trajectory1d_generator) {
  GenerateTrajectory1dPairs(init_s, init_d);
}

void BackupTrajectoryGenerator::GenerateTrajectory1dPairs(const State& init_s,
                                                          const State& init_d) {
  std::vector<std::shared_ptr<Curve1d>> lon_trajectories;
  std::array<double, 5> dds_condidates = {-0.1, -1.0, -2.0, -3.0, -4.0};
  for (const auto dds : dds_condidates) {
    lon_trajectories.emplace_back(
        new ConstantDecelerationTrajectory1d(init_s[0], init_s[1], dds));
  }

  std::vector<std::shared_ptr<Curve1d>> lat_trajectories;
  ptr_trajectory1d_generator_->GenerateLateralTrajectoryBundle(
      &lat_trajectories);

  for (auto& lon : lon_trajectories) {
    for (auto& lat : lat_trajectories) {
      trajectory_pair_pqueue_.emplace(lon, lat);
    }
  }
}

DiscretizedTrajectory BackupTrajectoryGenerator::GenerateTrajectory(
    const std::vector<points::PathPoint>& discretized_ref_points) {
  while (trajectory_pair_pqueue_.size() > 1) {
    auto top_pair = trajectory_pair_pqueue_.top();
    trajectory_pair_pqueue_.pop();
    DiscretizedTrajectory trajectory =
        TrajectoryCombiner::Combine(discretized_ref_points, *top_pair.first,
                                    *top_pair.second, init_relative_time_);
    if (!ptr_collision_checker_->InCollision(trajectory)) {
      return trajectory;
    }
  }
  auto top_pair = trajectory_pair_pqueue_.top();
  return TrajectoryCombiner::Combine(discretized_ref_points, *top_pair.first,
                                     *top_pair.second, init_relative_time_);
}
