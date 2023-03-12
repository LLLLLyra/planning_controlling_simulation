#include "constant_deceleration_trajectory1d.h"

#include <cmath>

#include "common/config/flags.h"
#include "glog/logging.h"

ConstantDecelerationTrajectory1d::ConstantDecelerationTrajectory1d(
    const double init_s, const double init_v, const double a)
    : init_s_(init_s), init_v_(init_v), deceleration_(-a) {
  if (init_v_ < -FLAGS_numerical_epsilon) {
    LOG(ERROR) << "negative init v = " << init_v_;
  }
  init_v_ = std::fabs(init_v_);
  CHECK(deceleration_ > 0.0);
  end_t_ = init_v_ / deceleration_;
  end_s_ = init_v_ * init_v_ / (2.0 * deceleration_) + init_s_;
}

double ConstantDecelerationTrajectory1d::Evaluate_s(const double t) const {
  if (t < end_t_) {
    double curr_v = init_v_ - deceleration_ * t;
    double delta_s = (curr_v + init_v_) * t * 0.5;
    return init_s_ + delta_s;
  } else {
    return end_s_;
  }
}

double ConstantDecelerationTrajectory1d::Evaluate_v(const double t) const {
  if (t < end_t_) {
    return init_v_ - deceleration_ * t;
  } else {
    return 0.0;
  }
}

double ConstantDecelerationTrajectory1d::Evaluate_a(const double t) const {
  if (t < end_t_) {
    return -deceleration_;
  } else {
    return 0.0;
  }
}

double ConstantDecelerationTrajectory1d::Evaluate_j(const double t) const {
  return 0.0;
}

double ConstantDecelerationTrajectory1d::ParamLength() const { return end_t_; }

std::string ConstantDecelerationTrajectory1d::ToString() const { return ""; }

double ConstantDecelerationTrajectory1d::Evaluate(const std::uint32_t order,
                                                  const double param) const {
  switch (order) {
    case 0:
      return Evaluate_s(param);
    case 1:
      return Evaluate_v(param);
    case 2:
      return Evaluate_a(param);
    case 3:
      return Evaluate_j(param);
  }
  return 0.0;
}
