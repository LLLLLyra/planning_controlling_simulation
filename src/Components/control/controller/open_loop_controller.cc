#include "open_loop_controller.h"

#include "common/config/log_path_conf.h"
#include "common/math/math_utils.h"
#include "common/proto/open_loop_controller_conf.pb.h"
#include "glog/logging.h"

OpenLoopController::OpenLoopController() {
  if (FLAGS_enable_csv_debug) {
    time_t rawtime;
    char name_buffer[150];
    std::time(&rawtime);
    std::tm time_tm;
    localtime_r(&rawtime, &time_tm);
    const std::string log_name =
        FLAGS_controller_log + std::string("open_loop_log_%F_%H%M%S.csv");
    strftime(name_buffer, sizeof(name_buffer), log_name.data(), &time_tm);
    open_loop_log_file_ = fopen(name_buffer, "w");
    if (open_loop_log_file_ == nullptr) {
      LOG(ERROR) << "Fail to open file:" << name_buffer;
    } else {
      fprintf(open_loop_log_file_,
              "s,"
              "lateral_error,"
              "speed,"
              "steer_angle,"
              "target_steer,"
              "feed_forward_steer,"
              "target_kappa,"
              "target_heading,"
              "target_x,"
              "target_y,"
              "current_x,"
              "current_y,"
              "current_heading"
              "\r\n");
      fflush(open_loop_log_file_);
    }
  }
}

void OpenLoopController::CloseFile() {
  if (FLAGS_enable_csv_debug) {
    if (open_loop_log_file_) {
      fclose(open_loop_log_file_);
      open_loop_log_file_ = nullptr;
    }
  }
}

Status OpenLoopController::Reset() {
  reached_ = false;
  return Status::OK();
}

OpenLoopController::~OpenLoopController() { CloseFile(); }

void OpenLoopController::Stop() { CloseFile(); }

Status OpenLoopController::Init(std::shared_ptr<DependencyInjector> injector,
                                const controller::ControlConf *control_conf) {
  injector_ = injector;
  speed_controller_input_limit_ =
      control_conf->open_loop_controller_conf().speed_controller_input_limit();
  k_ = control_conf->open_loop_controller_conf().k();
  return Status::OK();
}

Status OpenLoopController::ComputeControlCommand(
    const controller::LocalizationEstimate *localization,
    const canbus::Chassis *chassis, const planning::ADCTrajectory *trajectory,
    controller::ControlCommand *cmd) {
  trajectory_ = *trajectory;
  trajectory_analyzer_ = std::move(TrajectoryAnalyzer(&trajectory_));
  auto vehicle_state = injector_->vehicle_state();
  int gear = trajectory->gear() != canbus::Chassis::GEAR_REVERSE;
  GetTargetPoint();

  double speed = Clamp(target_point_.v(), -speed_controller_input_limit_,
                       speed_controller_input_limit_);
  const double steer_limit =
      vehicle_param_.max_steer_angle() / vehicle_param_.steer_ratio();
  const double wheel_base = vehicle_param_.wheel_base();
  double steer_angle = 0;
  double feed_forward_steer = std::atan(wheel_base * matched_point_.kappa());
  steer_angle += feed_forward_steer;
  steer_angle += k_ *
                 AngleDiff(vehicle_state->heading(), matched_point_.theta()) *
                 (gear ? 1.0 : -1.0);

  steer_angle = Clamp(steer_angle, -steer_limit, steer_limit);
  steer_angle = steer_angle / steer_limit * 100.0;

  speed = gear ? std::max(0.1, speed) : std::min(-0.1, speed);
  cmd->set_speed(speed);
  cmd->set_steering_target(steer_angle);

  if (FLAGS_enable_csv_debug && open_loop_log_file_) {
    fprintf(
        open_loop_log_file_,
        "%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,\r\n",
        s_matched_, d_matched_, speed, steer_angle, target_point_.steer(),
        feed_forward_steer, matched_point_.kappa(),
        target_point_.path_point().theta(), target_point_.path_point().x(),
        target_point_.path_point().y(), vehicle_state->x(), vehicle_state->y(),
        vehicle_state->heading());
  }

  return Status::OK();
}

void OpenLoopController::GetTargetPoint() {
  auto vehicle_state = injector_->vehicle_state();
  matched_point_ = trajectory_analyzer_.QueryMatchedPathPoint(
      vehicle_state->x(), vehicle_state->y());
  auto goal = trajectory_analyzer_.trajectory_points().back();
  trajectory_analyzer_.ToTrajectoryFrame(
      vehicle_state->x(), vehicle_state->y(), vehicle_state->heading(),
      vehicle_state->linear_velocity(), matched_point_, &s_matched_,
      &s_dot_matched_, &d_matched_, &d_dot_matched_);

  target_point_ = trajectory_analyzer_.QueryNearestPointByPosition(
      vehicle_state->x(), vehicle_state->y());
  double dx = goal.path_point().x() - vehicle_state->x();
  double dy = goal.path_point().y() - vehicle_state->y();
  bool distance_check = (dx * dx + dy * dy) < 0.0025;
  bool overtake_check =
      std::fabs(s_matched_) > std::fabs(goal.path_point().s());
  reached_ = distance_check || overtake_check;
}