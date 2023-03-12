#include "feasible_region.h"

#include <cmath>

#include "common/config/flags.h"
#include "glog/logging.h"

FeasibleRegion::FeasibleRegion(const std::array<double, 3>& init_s) {
  init_s_ = init_s;

  double v = init_s[1];
  CHECK_GE(v, 0.0);

  const double max_deceleration = -FLAGS_longitudinal_acceleration_lower_bound;
  t_at_zero_speed_ = v / max_deceleration;
  s_at_zero_speed_ = init_s[0] + v * v / (2.0 * max_deceleration);
}

double FeasibleRegion::SUpper(const double t) const {
  CHECK(t >= 0.0);
  return init_s_[0] + init_s_[1] * t +
         0.5 * FLAGS_longitudinal_acceleration_upper_bound * t * t;
}

double FeasibleRegion::SLower(const double t) const {
  if (t < t_at_zero_speed_) {
    return init_s_[0] + init_s_[1] * t +
           0.5 * FLAGS_longitudinal_acceleration_lower_bound * t * t;
  }
  return s_at_zero_speed_;
}

double FeasibleRegion::VUpper(const double t) const {
  return init_s_[1] + FLAGS_longitudinal_acceleration_upper_bound * t;
}

double FeasibleRegion::VLower(const double t) const {
  return t < t_at_zero_speed_
             ? init_s_[1] + FLAGS_longitudinal_acceleration_lower_bound * t
             : 0.0;
}

double FeasibleRegion::TLower(const double s) const {
  CHECK(s >= init_s_[0]);

  double delta_s = s - init_s_[0];
  double v = init_s_[1];
  double a = FLAGS_longitudinal_acceleration_upper_bound;
  double t = (std::sqrt(v * v + 2.0 * a * delta_s) - v) / a;
  return t;
}
