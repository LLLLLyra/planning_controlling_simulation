#include "stanley_controller.h"

#include "common/math/math_utils.h"
#include "common/proto/stanley_controller_conf.pb.h"
#include "glog/logging.h"

StanleyController::StanleyController() {
  if (FLAGS_enable_csv_debug) {
    time_t rawtime;
    char name_buffer[150];
    std::time(&rawtime);
    std::tm time_tm;
    localtime_r(&rawtime, &time_tm);
    strftime(name_buffer, 80, "../log/stanley_log__%F_%H%M%S.csv", &time_tm);
    stanley_log_ = fopen(name_buffer, "w");
    if (stanley_log_ == nullptr) {
      LOG(ERROR) << "Fail to open file:" << name_buffer;
    } else {
      fprintf(stanley_log_,
              "station_error,"
              "lateral_error,"
              "current_speed,"
              "current_heading,"
              "heading_error,"
              "current_steer_wheel_angle,"
              "speed,"
              "steer_angle,"
              "target_x,"
              "target_y,"
              "current_x,"
              "current_y,"
              "delta_e,"
              "delta_phi"
              "\r\n");
      fflush(stanley_log_);
    }
  }
}

void StanleyController::CloseFile() {
  if (FLAGS_enable_csv_debug) {
    if (stanley_log_) {
      fclose(stanley_log_);
      stanley_log_ = nullptr;
    }
  }
}

StanleyController::~StanleyController() { CloseFile(); }

void StanleyController::Stop() { CloseFile(); }

Status StanleyController::Init(std::shared_ptr<DependencyInjector> injector,
                               const controller::ControlConf *control_conf) {
  control_conf_ = control_conf;
  if (control_conf_ == nullptr) {
    LOG(ERROR) << "StanleyControllerParam nullptr";
    return Status(controller::ErrorCode::CONTROL_INIT_ERROR,
                  "Failed to load LonController conf");
  }
  injector_ = injector;
  const controller::StanleyControllerConf &stanley_conf =
      control_conf_->stanley_controller_conf();

  dt_ = stanley_conf.dt();
  k_ = stanley_conf.k();
  speed_controller_input_limit_ = stanley_conf.speed_controller_input_limit();

  station_pid_controller_.Init(stanley_conf.station_pid_conf());

  InitializeFilters(control_conf);

  return Status::OK();
}

void StanleyController::InitializeFilters(
    const controller::ControlConf *control_conf) {
  // Low pass filter
  std::vector<double> den(3, 0.0);
  std::vector<double> num(3, 0.0);
  LpfCoefficients(dt_, control_conf->stanley_controller_conf().cutoff_freq(),
                  &den, &num);
  digital_filter_.set_coefficients(den, num);
}

Status StanleyController::Reset() {
  station_pid_controller_.Reset();
  s_ = 0;
  reached_ = false;
  return Status::OK();
}

void StanleyController::GetTargetPoint() {
  auto vehicle_state = injector_->vehicle_state();
  matched_point_ = trajectory_analyzer_.QueryMatchedPathPoint(
      vehicle_state->x(), vehicle_state->y());
  auto goal = trajectory_analyzer_.trajectory_points().back();
  trajectory_analyzer_.ToTrajectoryFrame(
      vehicle_state->x(), vehicle_state->y(), vehicle_state->heading(),
      vehicle_state->linear_velocity(), matched_point_, &s_matched_,
      &s_dot_matched_, &d_matched_, &d_dot_matched_);

  size_t target_index = trajectory_analyzer_.QueryIndexByPosition(
      vehicle_state->x(), vehicle_state->y(), 0.1);
  target_point_ = trajectory_analyzer_.trajectory_points().at(target_index);
  station_error_ = target_point_.path_point().s() - s_matched_;
  double dx = goal.path_point().x() - vehicle_state->x();
  double dy = goal.path_point().y() - vehicle_state->y();
  bool distance_check = (dx * dx + dy * dy) < 0.0025;
  bool overtake_check =
      std::abs(std::abs(s_matched_) - std::abs(goal.path_point().s())) < 1e-2;
  reached_ = distance_check || overtake_check;
}

Status StanleyController::ComputeControlCommand(
    const controller::LocalizationEstimate *localization,
    const canbus::Chassis *chassis,
    const planning::ADCTrajectory *planning_published_trajectory,
    controller::ControlCommand *cmd) {
  localization_ = localization;
  chassis_ = chassis;

  auto debug = cmd->mutable_debug()->mutable_stanley_debug();
  debug->Clear();
  trajectory_ = *planning_published_trajectory;
  auto vehicle_state = injector_->vehicle_state();

  trajectory_analyzer_ = std::move(TrajectoryAnalyzer(&trajectory_));

  GetTargetPoint();

  double speed = station_pid_controller_.Control(station_error_, dt_);
  speed = Clamp(speed, -speed_controller_input_limit_,
                speed_controller_input_limit_);

  const double delta_phi = matched_point_.theta() - vehicle_state->heading();
  const double delta_e = std::atan(-k_ * d_matched_ / speed);
  double steer_wheel_angle = NormalizeAngle(delta_e + delta_phi);
  steer_wheel_angle *=
      trajectory_.gear() == canbus::Chassis::GEAR_REVERSE ? -1.0 : 1.0;
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
  debug->set_lateral_error(d_matched_);
  debug->set_station_error(station_error_);
  debug->set_current_speed(chassis_->speed_mps());
  debug->set_heading_error(delta_e + delta_phi);
  debug->set_speed_cmd(speed);
  debug->set_steer_angle_cmd(steer_angle);
  if (FLAGS_enable_csv_debug && stanley_log_) {
    fprintf(stanley_log_,
            "%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f, "
            "%.6f,\r\n",
            debug->station_error(), debug->lateral_error(),
            debug->current_speed(), debug->current_heading(),
            debug->heading_error(), debug->current_steer_wheel_angle(),
            debug->speed_cmd(), debug->steer_angle_cmd(),
            target_point_.path_point().x(), target_point_.path_point().y(),
            vehicle_state->x(), vehicle_state->y(), delta_e, delta_phi);
  }
  return Status::OK();
}
