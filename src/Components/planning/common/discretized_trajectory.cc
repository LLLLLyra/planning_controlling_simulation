#include "discretized_trajectory.h"

#include <limits>

#include "glog/logging.h"
#include "common/math/linear_interpolation.h"




DiscretizedTrajectory::DiscretizedTrajectory(
    const std::vector<points::TrajectoryPoint>& trajectory_points)
    : std::vector<points::TrajectoryPoint>(trajectory_points) {
  CHECK(!trajectory_points.empty())
      << "trajectory_points should NOT be empty()";
}


points::TrajectoryPoint DiscretizedTrajectory::Evaluate(
    const double relative_time) const {
  auto comp = [](const points::TrajectoryPoint& p, const double relative_time) {
    return p.relative_time() < relative_time;
  };

  auto it_lower = std::lower_bound(begin(), end(), relative_time, comp);

  if (it_lower == begin()) {
    return front();
  } else if (it_lower == end()) {
    LOG(INFO) << "When evaluate trajectory, relative_time(" << relative_time
          << ") is too large";
    return back();
  }
  return InterpolateUsingLinearApproximation(
      *(it_lower - 1), *it_lower, relative_time);
}

size_t DiscretizedTrajectory::QueryLowerBoundPoint(const double relative_time,
                                                   const double epsilon) const {
  CHECK(!empty());

  if (relative_time >= back().relative_time()) {
    return size() - 1;
  }
  auto func = [&epsilon](const points::TrajectoryPoint& tp,
                         const double relative_time) {
    return tp.relative_time() + epsilon < relative_time;
  };
  auto it_lower = std::lower_bound(begin(), end(), relative_time, func);
  return std::distance(begin(), it_lower);
}

size_t DiscretizedTrajectory::QueryNearestPoint(
    const Vec2d& position) const {
  double dist_sqr_min = std::numeric_limits<double>::max();
  size_t index_min = 0;
  for (size_t i = 0; i < size(); ++i) {
    const Vec2d curr_point(data()[i].path_point().x(),
                                         data()[i].path_point().y());

    const double dist_sqr = curr_point.DistanceSquareTo(position);
    if (dist_sqr < dist_sqr_min) {
      dist_sqr_min = dist_sqr;
      index_min = i;
    }
  }
  return index_min;
}

size_t DiscretizedTrajectory::QueryNearestPointWithBuffer(
    const Vec2d& position, const double buffer) const {
  double dist_sqr_min = std::numeric_limits<double>::max();
  size_t index_min = 0;
  for (size_t i = 0; i < size(); ++i) {
    const Vec2d curr_point(data()[i].path_point().x(),
                                         data()[i].path_point().y());

    const double dist_sqr = curr_point.DistanceSquareTo(position);
    if (dist_sqr < dist_sqr_min + buffer) {
      dist_sqr_min = dist_sqr;
      index_min = i;
    }
  }
  return index_min;
}

void DiscretizedTrajectory::AppendTrajectoryPoint(
    const points::TrajectoryPoint& trajectory_point) {
  if (!empty()) {
    CHECK_GT(trajectory_point.relative_time(), back().relative_time());
  }
  push_back(trajectory_point);
}

const points::TrajectoryPoint& DiscretizedTrajectory::TrajectoryPointAt(
    const size_t index) const {
  CHECK_LT(index, NumOfPoints());
  return data()[index];
}

points::TrajectoryPoint DiscretizedTrajectory::StartPoint() const {
  CHECK(!empty());
  return front();
}

double DiscretizedTrajectory::GetTemporalLength() const {
  if (empty()) {
    return 0.0;
  }
  return back().relative_time() - front().relative_time();
}

double DiscretizedTrajectory::GetSpatialLength() const {
  if (empty()) {
    return 0.0;
  }
  return back().path_point().s() - front().path_point().s();
}

