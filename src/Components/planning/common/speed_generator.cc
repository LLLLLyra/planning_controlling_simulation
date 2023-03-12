#include "speed_generator.h"

#include "common/math/math_utils.h"
#include "common/math/vec2d.h"
#include "glog/logging.h"

bool SpeedGenerator::GenerateSCurveSpeedAcceleration(
    KineticResult* result, vehicle::VehicleParam vehicle_param,
    planning::PiecewiseJerkSpeedOptimizerConfig piecewise_conf) {
  CHECK_NOTNULL(result);
  if (result->x.size() < 2 || result->y.size() < 2 || result->phi.size() < 2) {
    LOG(ERROR)
        << "result size check when generating speed and acceleration fail";
    return false;
  }
  if (result->x.size() != result->y.size() ||
      result->x.size() != result->phi.size()) {
    LOG(ERROR) << "result sizes not equal";
    return false;
  }

  // get gear info
  double init_heading = result->phi.front();
  const Vec2d init_tracking_vector(result->x[1] - result->x[0],
                                   result->y[1] - result->y[0]);
  const double gear =
      std::abs(NormalizeAngle(init_heading - init_tracking_vector.Angle())) <
      M_PI_2;

  // get path lengh
  size_t path_points_size = result->x.size();

  double accumulated_s = 0.0;
  result->accumulated_s.clear();
  auto last_x = result->x.front();
  auto last_y = result->y.front();
  for (size_t i = 0; i < path_points_size; ++i) {
    double x_diff = result->x[i] - last_x;
    double y_diff = result->y[i] - last_y;
    accumulated_s += std::sqrt(x_diff * x_diff + y_diff * y_diff);
    result->accumulated_s.push_back(accumulated_s);
    last_x = result->x[i];
    last_y = result->y[i];
  }
  // assume static initial state
  const double init_v = 0.0;
  const double init_a = 0.0;

  // minimum time speed optimization
  const double max_forward_v = piecewise_conf.max_forward_v();
  const double max_reverse_v = piecewise_conf.max_reverse_v();
  const double max_forward_acc = piecewise_conf.max_forward_acc();
  const double max_reverse_acc = piecewise_conf.max_reverse_acc();
  const double max_acc_jerk = piecewise_conf.max_acc_jerk();
  const double delta_t = piecewise_conf.delta_t();

  SpeedData speed_data;

  // TODO(Jinyun): explore better time horizon heuristic
  const double path_length = result->accumulated_s.back();
  const double total_t = std::max(gear ? 1.5 *
                                             (max_forward_v * max_forward_v +
                                              path_length * max_forward_acc) /
                                             (max_forward_acc * max_forward_v)
                                       : 1.5 *
                                             (max_reverse_v * max_reverse_v +
                                              path_length * max_reverse_acc) /
                                             (max_reverse_acc * max_reverse_v),
                                  10.0);

  const size_t num_of_knots = static_cast<size_t>(total_t / delta_t) + 1;

  PiecewiseJerkSpeedProblem piecewise_jerk_problem(
      num_of_knots, delta_t, {0.0, std::abs(init_v), std::abs(init_a)});

  // set end constraints
  std::vector<std::pair<double, double>> x_bounds(num_of_knots,
                                                  {0.0, path_length});

  const double max_v = gear ? max_forward_v : max_reverse_v;
  const double max_acc = gear ? max_forward_acc : max_reverse_acc;

  const auto upper_dx = std::fmax(max_v, std::abs(init_v));
  std::vector<std::pair<double, double>> dx_bounds(num_of_knots,
                                                   {0.0, upper_dx});
  std::vector<std::pair<double, double>> ddx_bounds(num_of_knots,
                                                    {-max_acc, max_acc});

  x_bounds[num_of_knots - 1] = std::make_pair(path_length, path_length);
  dx_bounds[num_of_knots - 1] = std::make_pair(0.0, 0.0);
  ddx_bounds[num_of_knots - 1] = std::make_pair(0.0, 0.0);

  std::vector<double> x_ref(num_of_knots, path_length);
  piecewise_jerk_problem.set_x_ref(10000.0, std::move(x_ref));
  piecewise_jerk_problem.set_weight_ddx(piecewise_conf.acc_weight());
  piecewise_jerk_problem.set_weight_dddx(piecewise_conf.jerk_weight());
  piecewise_jerk_problem.set_x_bounds(std::move(x_bounds));
  piecewise_jerk_problem.set_dx_bounds(std::move(dx_bounds));
  piecewise_jerk_problem.set_ddx_bounds(std::move(ddx_bounds));
  piecewise_jerk_problem.set_dddx_bound(max_acc_jerk);

  // solve the problem
  if (!piecewise_jerk_problem.Optimize(40000)) {
    LOG(ERROR) << "Piecewise jerk speed optimizer failed!";
    return false;
  }

  // extract output
  const std::vector<double>& s = piecewise_jerk_problem.opt_x();
  const std::vector<double>& ds = piecewise_jerk_problem.opt_dx();
  const std::vector<double>& dds = piecewise_jerk_problem.opt_ddx();

  // assign speed point by gear
  speed_data.AppendSpeedPoint(s[0], 0.0, ds[0], dds[0], 0.0);
  const double kEpislon = 1.0e-6;
  const double sEpislon = 1.0e-6;
  for (size_t i = 1; i < num_of_knots; ++i) {
    if (s[i - 1] - s[i] > kEpislon) {
      LOG(INFO) << "unexpected decreasing s in speed smoothing at time "
                << static_cast<double>(i) * delta_t << "with total time "
                << total_t;
      break;
    }
    speed_data.AppendSpeedPoint(s[i], delta_t * static_cast<double>(i), ds[i],
                                dds[i], (dds[i] - dds[i - 1]) / delta_t);
    // cut the speed data when it is about to meet end condition
    if (path_length - s[i] < sEpislon) {
      break;
    }
  }

  // combine speed and path profile
  DiscretizedPath path_data;
  for (size_t i = 0; i < path_points_size; ++i) {
    points::PathPoint path_point;
    path_point.set_x(result->x[i]);
    path_point.set_y(result->y[i]);
    path_point.set_theta(result->phi[i]);
    path_point.set_s(result->accumulated_s[i]);
    path_data.push_back(std::move(path_point));
  }

  KineticResult combined_result;

  // TODO(Jinyun): move to confs
  const double kDenseTimeResoltuion = piecewise_conf.k_dense_time_resoltuion();
  const double time_horizon =
      speed_data.TotalTime() + kDenseTimeResoltuion * 1.0e-6;
  if (path_data.empty()) {
    LOG(ERROR) << "path data is empty";
    return false;
  }
  for (double cur_rel_time = 0.0; cur_rel_time < time_horizon;
       cur_rel_time += kDenseTimeResoltuion) {
    points::SpeedPoint speed_point;
    if (!speed_data.EvaluateByTime(cur_rel_time, &speed_point)) {
      LOG(ERROR) << "Fail to get speed point with relative time "
                 << cur_rel_time;
      return false;
    }

    // limit speed
    if (std::fabs(speed_point.v()) < 1e-3) {
      speed_point.set_v(0);
    }

    if (speed_point.s() > path_data.Length()) {
      break;
    }

    points::PathPoint path_point = path_data.Evaluate(speed_point.s());

    combined_result.x.push_back(path_point.x());
    combined_result.y.push_back(path_point.y());
    combined_result.phi.push_back(path_point.theta());
    combined_result.accumulated_s.push_back(path_point.s());
    if (!gear) {
      combined_result.v.push_back(-speed_point.v());
      combined_result.a.push_back(-speed_point.a());
    } else {
      combined_result.v.push_back(speed_point.v());
      combined_result.a.push_back(speed_point.a());
    }
  }

  combined_result.a.pop_back();

  // recalc step size
  path_points_size = combined_result.x.size();

  // load steering from phi
  for (size_t i = 0; i + 1 < path_points_size; ++i) {
    double discrete_steer =
        (combined_result.phi[i + 1] - combined_result.phi[i]) *
        vehicle_param.wheel_base() /
        (combined_result.accumulated_s[i + 1] -
         combined_result.accumulated_s[i]);
    discrete_steer =
        gear ? std::atan(discrete_steer) : std::atan(-discrete_steer);
    combined_result.steer.push_back(discrete_steer);
  }

  *result = combined_result;
  return true;
}