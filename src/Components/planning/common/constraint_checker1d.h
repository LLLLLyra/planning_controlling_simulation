#ifndef _CONSTRAINT_CHECKER1D_H_
#define _CONSTRAINT_CHECKER1D_H_

#include "common/math/curve1d.h"
#include "discretized_trajectory.h"

class ConstraintChecker1d {
 public:
  ConstraintChecker1d() = delete;

  static bool IsValidLongitudinalTrajectory(const Curve1d& lon_trajectory);

  static bool IsValidLateralTrajectory(const Curve1d& lat_trajectory,
                                       const Curve1d& lon_trajectory);
};

#endif
