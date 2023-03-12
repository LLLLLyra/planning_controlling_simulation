#ifndef _PIECEWISE_JERK_TRAJECTORY1D_H_
#define _PIECEWISE_JERK_TRAJECTORY1D_H_

#define FLAGS_numerical_epsilon 1e-6

#include <string>
#include <vector>

#include "common/math/curve1d.h"
#include "constant_jerk_trajectory1d.h"

class PiecewiseJerkTrajectory1d : public Curve1d {
 public:
  PiecewiseJerkTrajectory1d(const double p, const double v, const double a);

  virtual ~PiecewiseJerkTrajectory1d() = default;

  double Evaluate(const std::uint32_t order, const double param) const;

  double ParamLength() const;

  std::string ToString() const;

  void AppendSegment(const double jerk, const double param);

 private:
  std::vector<ConstantJerkTrajectory1d> segments_;

  double last_p_;

  double last_v_;

  double last_a_;

  std::vector<double> param_;
};

#endif
