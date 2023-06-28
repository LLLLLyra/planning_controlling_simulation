#include "standing_still_trajectory1d.h"



StandingStillTrajectory1d::StandingStillTrajectory1d(const double p,
                                                     const double duration)
    : fixed_position_(p), duration_(duration) {}

double StandingStillTrajectory1d::ParamLength() const { return duration_; }

std::string StandingStillTrajectory1d::ToString() const { return ""; }

double StandingStillTrajectory1d::Evaluate(const std::uint32_t order,
                                           const double param) const {
  //  ACHECK(param <= duration_);
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

double StandingStillTrajectory1d::Evaluate_s(const double t) const {
  return fixed_position_;
}

double StandingStillTrajectory1d::Evaluate_v(const double t) const {
  return 0.0;
}

double StandingStillTrajectory1d::Evaluate_a(const double t) const {
  return 0.0;
}

double StandingStillTrajectory1d::Evaluate_j(const double t) const {
  return 0.0;
}

