#ifndef _SPEED_GENERATOR_H_
#define _SPEED_GENERATOR_H_

#include <vector>

#include "common/math/discretized_path.h"
#include "common/math/piecewise_jerk/piecewise_jerk_speed_problem.h"
#include "common/math/speed_data.h"
#include "common/proto/planner_config.pb.h"
#include "common/proto/vehicle_config.pb.h"

struct KineticResult {
  std::vector<double> x;
  std::vector<double> y;
  std::vector<double> phi;
  std::vector<double> v;
  std::vector<double> a;
  std::vector<double> steer;
  std::vector<double> accumulated_s;
};

class SpeedGenerator {
 public:
  SpeedGenerator() = default;
  ~SpeedGenerator() = default;
  static bool GenerateSCurveSpeedAcceleration(
      KineticResult* result, vehicle::VehicleParam vehicle_param,
      planning::PiecewiseJerkSpeedOptimizerConfig piecewise_conf);
};

#endif