#ifndef _TRAJECTORY_COMBINER_H_
#define _TRAJECTORY_COMBINER_H_

#define FLAGS_numerical_epsilon 1e-6
#define FLAGS_trajectory_time_length 8.0
#define FLAGS_trajectory_time_resolution 0.1

#include <vector>

#include "common/math/curve1d.h"
#include "planning/common/discretized_trajectory.h"

class TrajectoryCombiner {
 public:
  static DiscretizedTrajectory Combine(
      const std::vector<points::PathPoint>& reference_line,
      const Curve1d& lon_trajectory, const Curve1d& lat_trajectory,
      const double init_relative_time);
};

#endif
