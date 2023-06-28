#include "constraint_checker1d.h"

#include "common/config/flags.h"
namespace {

inline bool fuzzy_within(const double v, const double lower, const double upper,
                         const double e = 1.0e-4) {
  return v > lower - e && v < upper + e;
}
}  // namespace

bool ConstraintChecker1d::IsValidLongitudinalTrajectory(
    const Curve1d& lon_trajectory) {
  double t = 0.0;
  while (t < lon_trajectory.ParamLength()) {
    double v = lon_trajectory.Evaluate(1, t);  // evaluate_v
    if (!fuzzy_within(v, FLAGS_speed_lower_bound, FLAGS_speed_upper_bound)) {
      return false;
    }

    double a = lon_trajectory.Evaluate(2, t);  // evaluate_a
    if (!fuzzy_within(a, FLAGS_longitudinal_acceleration_lower_bound,
                      FLAGS_longitudinal_acceleration_upper_bound)) {
      return false;
    }

    double j = lon_trajectory.Evaluate(3, t);
    if (!fuzzy_within(j, FLAGS_longitudinal_jerk_lower_bound,
                      FLAGS_longitudinal_jerk_upper_bound)) {
      return false;
    }
    t += FLAGS_trajectory_time_resolution;
  }
  return true;
}

bool ConstraintChecker1d::IsValidLateralTrajectory(
    const Curve1d& lat_trajectory, const Curve1d& lon_trajectory) {
  double t = 0.0;
  while (t < lon_trajectory.ParamLength()) {
    double s = lon_trajectory.Evaluate(0, t);
    double dd_ds = lat_trajectory.Evaluate(1, s);
    double ds_dt = lon_trajectory.Evaluate(1, t);

    double d2d_ds2 = lat_trajectory.Evaluate(2, s);
    double d2s_dt2 = lon_trajectory.Evaluate(2, t);

    double a = 0.0;
    if (s < lat_trajectory.ParamLength()) {
      a = d2d_ds2 * ds_dt * ds_dt + dd_ds * d2s_dt2;
    }

    if (!fuzzy_within(a, -FLAGS_lateral_acceleration_bound,
                      FLAGS_lateral_acceleration_bound)) {
      return false;
    }

    // this is not accurate, just an approximation...
    double j = 0.0;
    if (s < lat_trajectory.ParamLength()) {
      j = lat_trajectory.Evaluate(3, s) * lon_trajectory.Evaluate(3, t);
    }

    if (!fuzzy_within(j, -FLAGS_lateral_jerk_bound, FLAGS_lateral_jerk_bound)) {
      return false;
    }
    t += FLAGS_trajectory_time_resolution;
  }
  return true;
}
