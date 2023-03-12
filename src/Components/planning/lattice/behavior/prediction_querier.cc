#include "prediction_querier.h"

#include "common/math/linear_interpolation.h"
#include "common/math/path_matcher.h"

PredictionQuerier::PredictionQuerier(
    const std::vector<const Obstacle*>& obstacles,
    const std::shared_ptr<std::vector<points::PathPoint>>& ptr_reference_line)
    : ptr_reference_line_(ptr_reference_line) {
  for (const auto ptr_obstacle : obstacles) {
    if (InsertIfNotPresent(&id_obstacle_map_, ptr_obstacle->Id(),
                           ptr_obstacle)) {
      obstacles_.push_back(ptr_obstacle);
    } else {
      LOG(INFO) << "Duplicated obstacle found [" << ptr_obstacle->Id() << "]";
    }
  }
}

std::vector<const Obstacle*> PredictionQuerier::GetObstacles() const {
  return obstacles_;
}

double PredictionQuerier::ProjectVelocityAlongReferenceLine(
    const std::string& obstacle_id, const double s, const double t) const {
  CHECK(id_obstacle_map_.find(obstacle_id) != id_obstacle_map_.end());

  const auto& trajectory = id_obstacle_map_.at(obstacle_id)->Trajectory();
  int num_traj_point = trajectory.trajectory_point_size();
  if (num_traj_point < 2) {
    return 0.0;
  }

  if (t < trajectory.trajectory_point(0).relative_time() ||
      t > trajectory.trajectory_point(num_traj_point - 1).relative_time()) {
    return 0.0;
  }

  auto matched_it =
      std::lower_bound(trajectory.trajectory_point().begin(),
                       trajectory.trajectory_point().end(), t,
                       [](const points::TrajectoryPoint& p, const double t) {
                         return p.relative_time() < t;
                       });

  double v = matched_it->v();
  double theta = matched_it->path_point().theta();
  double v_x = v * std::cos(theta);
  double v_y = v * std::sin(theta);

  points::PathPoint obstacle_point_on_ref_line =
      PathMatcher::MatchToPath(*ptr_reference_line_, s);
  auto ref_theta = obstacle_point_on_ref_line.theta();

  return std::cos(ref_theta) * v_x + std::sin(ref_theta) * v_y;
}
