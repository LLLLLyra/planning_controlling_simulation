#include "simple_pid_lat_lon_controller.h"

#include "common/config/log_path_conf.h"
#include "glog/logging.h"

SimplePIDLatLonController::SimplePIDLatLonController() {
  if (FLAGS_enable_csv_debug) {
    time_t rawtime;
    char name_buffer[150];
    std::time(&rawtime);
    std::tm time_tm;
    localtime_r(&rawtime, &time_tm);
    const std::string log_name =
        FLAGS_controller_log + std::string("speed_log__%F_%H%M%S.csv");
    strftime(name_buffer, sizeof(name_buffer), log_name.data(), &time_tm);
    sim_pid_log_file_ = fopen(name_buffer, "w");
    if (sim_pid_log_file_ == nullptr) {
      LOG(ERROR) << "Fail to open file:" << name_buffer;
    } else {
      fprintf(sim_pid_log_file_,
              "look_ahead_station,"
              "station_error,"
              "lateral_error,"
              "current_speed,"
              "current_heading,"
              "heading_error,"
              "current_steer_wheel_angle,"
              "speed,"
              "steer_angle,"
              "look_ahead,"
              "target_x,"
              "target_y,"
              "current_x,"
              "current_y"
              "\r\n");
      fflush(sim_pid_log_file_);
    }
  }
}

void SimplePIDLatLonController::CloseFile() {
  if (FLAGS_enable_csv_debug) {
    if (sim_pid_log_file_) {
      fclose(sim_pid_log_file_);
      sim_pid_log_file_ = nullptr;
    }
  }
}

SimplePIDLatLonController::~SimplePIDLatLonController() { CloseFile(); }

void SimplePIDLatLonController::Stop() { CloseFile(); }

Status SimplePIDLatLonController::Init(
    std::shared_ptr<DependencyInjector> injector,
    const controller::ControlConf *control_conf) {
  control_conf_ = control_conf;
  if (control_conf_ == nullptr) {
    LOG(ERROR) << "SimplePIDLatLonControllerParam nullptr";
    return Status(controller::ErrorCode::CONTROL_INIT_ERROR,
                  "Failed to load LonController conf");
  }
  injector_ = injector;
  const controller::SimplePIDLatLonControllerConf &simple_pid_lat_lon_conf =
      control_conf_->simple_pid_lat_lon_controller_conf();

  look_ahead_distance_ = simple_pid_lat_lon_conf.look_ahead_distance();
  dt_ = simple_pid_lat_lon_conf.dt();
  speed_controller_input_limit_ =
      simple_pid_lat_lon_conf.speed_controller_input_limit();

  station_pid_controller_.Init(simple_pid_lat_lon_conf.station_pid_conf());
  yaw_pid_controller_.Init(simple_pid_lat_lon_conf.yaw_pid_conf());

  InitializeFilters(control_conf);

  return Status::OK();
}

void SimplePIDLatLonController::InitializeFilters(
    const controller::ControlConf *control_conf) {
  // Low pass filter
  std::vector<double> den(3, 0.0);
  std::vector<double> num(3, 0.0);
  LpfCoefficients(
      dt_, control_conf->simple_pid_lat_lon_controller_conf().cutoff_freq(),
      &den, &num);
  digital_filter_.set_coefficients(den, num);
  yaw_error_filter_ = MeanFilter(static_cast<std::uint_fast8_t>(
      control_conf->simple_pid_lat_lon_controller_conf()
          .mean_filter_window_size()));
}

Status SimplePIDLatLonController::Reset() {
  station_pid_controller_.Reset();
  yaw_pid_controller_.Reset();
  s_matched_ = 0;
  return Status::OK();
}

Status SimplePIDLatLonController::ComputeControlCommand(
    const controller::LocalizationEstimate *localization,
    const canbus::Chassis *chassis,
    const planning::ADCTrajectory *planning_published_trajectory,
    controller::ControlCommand *cmd) {
  localization_ = localization;
  chassis_ = chassis;

  auto debug = cmd->mutable_debug()->mutable_simple_pid_lat_lon_debug();
  debug->Clear();
  trajectory_ = *planning_published_trajectory;
  trajectory_analyzer_ = std::move(TrajectoryAnalyzer(&trajectory_));
  auto vehicle_state = injector_->vehicle_state();

  target_point_ =
      GetTargetPoint(vehicle_state->x(), vehicle_state->y(),
                     trajectory_.gear() != canbus::Chassis::GEAR_REVERSE,
                     vehicle_state->heading());

  double speed = station_pid_controller_.Control(station_error_, dt_);
  speed = Clamp(speed, -speed_controller_input_limit_,
                speed_controller_input_limit_);
  if (planning_published_trajectory->gear() == canbus::Chassis::GEAR_REVERSE) {
    speed = speed <= 0 ? speed : -0.01;
  } else {
    speed = speed >= 0 ? speed : 0.01;
  }
  yaw_error_ =
      NormalizeAngle(
          std::atan2(target_point_.path_point().y() - vehicle_state->y(),
                     target_point_.path_point().x() - vehicle_state->x()) -
          vehicle_state->heading()) *
      (chassis_->gear_location() != canbus::Chassis::GEAR_REVERSE ? 1 : -1);
  yaw_error_ = yaw_error_filter_.Update(yaw_error_);
  double steer_wheel_angle =
      NormalizeAngle(yaw_pid_controller_.Control(yaw_error_, dt_));
  double steer_angle = steer_wheel_angle * vehicle_param_.steer_ratio();
  steer_angle = digital_filter_.Filter(steer_angle);
  steer_angle = Clamp(steer_angle, -vehicle_param_.max_steer_angle(),
                      vehicle_param_.max_steer_angle());
  steer_angle = steer_angle / vehicle_param_.max_steer_angle() * 100;

  cmd->set_speed(speed);
  cmd->set_steering_target(steer_angle);

  debug->mutable_target_point()->mutable_path_point()->set_x(
      target_point_.path_point().x());
  debug->mutable_target_point()->mutable_path_point()->set_y(
      target_point_.path_point().y());
  debug->mutable_target_point()->mutable_path_point()->set_theta(
      target_point_.path_point().theta());
  debug->mutable_target_point()->set_v(target_point_.v());
  debug->set_current_heading(vehicle_state->heading());
  debug->set_look_ahead_station(s_matched_ + station_error_);
  debug->set_station_error(station_error_);
  debug->set_current_speed(chassis_->speed_mps());
  debug->set_heading_error(yaw_error_);
  debug->set_speed_cmd(speed);
  debug->set_steer_angle_cmd(steer_angle);
  if (FLAGS_enable_csv_debug && sim_pid_log_file_) {
    fprintf(sim_pid_log_file_,
            "%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,"
            "%.6f,\r\n",
            debug->look_ahead_station(), debug->station_error(), d_matched_,
            debug->current_speed(), debug->current_heading(),
            debug->heading_error(), debug->current_steer_wheel_angle(),
            debug->speed_cmd(), debug->steer_angle_cmd(),
            control_conf_->simple_pid_lat_lon_controller_conf()
                .look_ahead_distance(),
            target_point_.path_point().x(), target_point_.path_point().y(),
            vehicle_state->x(), vehicle_state->y());
  }
  return Status::OK();
}

double SimplePIDLatLonController::GetDistance(const points::PathPoint &p1,
                                              const points::PathPoint &p2) {
  return std::sqrt(std::pow(p1.x() - p2.x(), 2) + std::pow(p1.y() - p2.y(), 2));
}

points::TrajectoryPoint SimplePIDLatLonController::GetTargetPoint(
    const double x, const double y, const bool forward, const double theta) {
  points::PathPoint curr_path_point;
  curr_path_point.set_x(x);
  curr_path_point.set_y(y);

  // query the nearest point by position
  size_t index_min = trajectory_analyzer_.QueryNearestIndexByPosition(x, y);
  size_t target_index = index_min;
  target_point_ = *(trajectory_.trajectory_point().begin() + target_index);
  auto matched_point = trajectory_.trajectory_point().begin() + index_min;
  const size_t n = trajectory_.trajectory_point().size();
  double current_distance =
      GetDistance(matched_point->path_point(), target_point_.path_point());

  // check the target point is ahead of current path point
  Vec2d target_direction(forward ? std::cos(theta) : -std::cos(theta),
                         forward ? std::sin(theta) : -std::sin(theta));
  Vec2d current_direction(target_point_.path_point().x() - x,
                          target_point_.path_point().y() - y);

  // skip the trajectory before the ramdom start
  while ((target_index < n - 1) && (current_distance < look_ahead_distance_)) {
    ++target_index;
    target_point_ = *(trajectory_.trajectory_point().begin() + target_index);
    current_distance =
        GetDistance(matched_point->path_point(), target_point_.path_point());
    current_direction.set_x(target_point_.path_point().x() - x);
    current_direction.set_y(target_point_.path_point().y() - y);
  }

  auto vehicle_state = injector_->vehicle_state();
  auto projection_point = trajectory_analyzer_.QueryMatchedPathPoint(
      vehicle_state->x(), vehicle_state->y());

  trajectory_analyzer_.ToTrajectoryFrame(
      vehicle_state->x(), vehicle_state->y(), vehicle_state->heading(),
      vehicle_state->linear_velocity(), projection_point, &s_matched_,
      &s_dot_matched_, &d_matched_, &d_dot_matched_);
  station_error_ = target_point_.path_point().s() - s_matched_;

  auto goal = trajectory_analyzer_.trajectory_points().back();
  double dx = goal.path_point().x() - vehicle_state->x();
  double dy = goal.path_point().y() - vehicle_state->y();
  bool distance_check = (dx * dx + dy * dy) < 0.0025;
  bool overtake_check =
      std::abs(std::abs(s_matched_) - std::abs(goal.path_point().s())) < 1e-2;
  reached_ = distance_check || overtake_check;
  return target_point_;
}
