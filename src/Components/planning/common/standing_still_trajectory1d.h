#ifndef _STANDING_STILL_TRAJECTORY1D_H_
#define _STANDING_STILL_TRAJECTORY1D_H_

#include <string>

#include "common/math/curve1d.h"


class StandingStillTrajectory1d : public Curve1d {
 public:
  StandingStillTrajectory1d(const double p, const double duration);

  virtual ~StandingStillTrajectory1d() = default;

  double ParamLength() const override;

  std::string ToString() const override;

  double Evaluate(const std::uint32_t order, const double param) const override;

 private:
  double Evaluate_s(const double t) const;

  double Evaluate_v(const double t) const;

  double Evaluate_a(const double t) const;

  double Evaluate_j(const double t) const;

 private:
  double fixed_position_;

  double duration_;
};


#endif
