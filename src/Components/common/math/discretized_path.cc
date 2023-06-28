
#include "discretized_path.h"

#include <algorithm>

#include "glog/logging.h"
#include "linear_interpolation.h"

DiscretizedPath::DiscretizedPath(std::vector<points::PathPoint> path_points)
    : std::vector<points::PathPoint>(std::move(path_points)) {}

double DiscretizedPath::Length() const {
  if (empty()) {
    return 0.0;
  }
  return back().s() - front().s();
}

points::PathPoint DiscretizedPath::Evaluate(const double path_s) const {
  CHECK(!empty());
  auto it_lower = QueryLowerBound(path_s);
  if (it_lower == begin()) {
    return front();
  }
  if (it_lower == end()) {
    return back();
  }
  return InterpolateUsingLinearApproximation(*(it_lower - 1), *it_lower,
                                             path_s);
}

std::vector<points::PathPoint>::const_iterator DiscretizedPath::QueryLowerBound(
    const double path_s) const {
  auto func = [](const points::PathPoint &tp, const double path_s) {
    return tp.s() < path_s;
  };
  return std::lower_bound(begin(), end(), path_s, func);
}

points::PathPoint DiscretizedPath::EvaluateReverse(const double path_s) const {
  CHECK(!empty());
  auto it_upper = QueryUpperBound(path_s);
  if (it_upper == begin()) {
    return front();
  }
  if (it_upper == end()) {
    return back();
  }
  return InterpolateUsingLinearApproximation(*(it_upper - 1), *it_upper,
                                             path_s);
}

std::vector<points::PathPoint>::const_iterator DiscretizedPath::QueryUpperBound(
    const double path_s) const {
  auto func = [](const double path_s, const points::PathPoint &tp) {
    return tp.s() < path_s;
  };
  return std::upper_bound(begin(), end(), path_s, func);
}
