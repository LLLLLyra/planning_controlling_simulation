#include "lat_lon_controller.h"

#include <algorithm>
#include <ctime>
#include <iomanip>
#include <utility>
#include <vector>

#include "Eigen/LU"
#include "common/config/log_path_conf.h"
#include "common/config/timer.h"
#include "common/math/linear_interpolation.h"
#include "common/math/linear_quadratic_regulator.h"
#include "common/math/math_utils.h"
#include "common/math/quaternion.h"
#include "glog/logging.h"

using controller::ErrorCode;
using points::TrajectoryPoint;
using Matrix = Eigen::MatrixXd;

constexpr double GRA_ACC = 9.8;

namespace {

std::string GetLogFileName() {
  time_t raw_time;
  char name_buffer[200];
  std::time(&raw_time);

  std::tm time_tm;
  localtime_r(&raw_time, &time_tm);
  const std::string log_name =
      FLAGS_controller_log + std::string("lat_lon_controller_%F_%H%M%S.csv");
  strftime(name_buffer, sizeof(name_buffer), log_name.data(), &time_tm);
  return std::string(name_buffer);
}

void WriteHeaders(std::ofstream &file_stream) {
  file_stream << "current_lateral_error,"
              << "current_ref_heading,"
              << "current_heading,"
              << "current_heading_error,"
              << "heading_error_rate,"
              << "lateral_error_rate,"
              << "current_curvature,"
              << "steer_angle,"
              << "steer_angle_feedforward,"
              << "steer_angle_lateral_contribution,"
              << "steer_angle_lateral_rate_contribution,"
              << "steer_angle_heading_contribution,"
              << "steer_angle_heading_rate_contribution,"
              << "steer_angle_feedback,"
              << "steering_position,"
              << "v,"
              << "station_reference,"
              << "station_error,"
              << "station_error_limited,"
              << "preview_station_error,"
              << "speed_reference,"
              << "speed_error,"
              << "speed_error_limited,"
              << "preview_speed_reference,"
              << "preview_speed_error,"
              << "preview_acceleration_reference,"
              << "acceleration_cmd_closeloop,"
              << "acceleration_cmd,"
              << "acceleration_lookup,"
              << "speed_lookup,"
              << "calibration_value,"
              << "throttle_cmd,"
              << "brake_cmd,"
              << "speed,"
              << "is_full_stop,"
              << "target_x,"
              << "target_y," << std::endl;
}
}  // namespace

LatLonController::LatLonController() : name_("Lat Lon Controller") {
  if (FLAGS_enable_csv_debug) {
    lat_lon_file_.open(GetLogFileName(), std::ofstream::out);
    lat_lon_file_ << std::fixed;
    lat_lon_file_ << std::setprecision(6);
    WriteHeaders(lat_lon_file_);
  }
  LOG(INFO) << "Using " << name_;
}

LatLonController::~LatLonController() { CloseLogFile(); }

void LatLonController::CloseLogFile() {
  if (FLAGS_enable_csv_debug && lat_lon_file_.is_open()) {
    lat_lon_file_.close();
  }
}

void LatLonController::ProcessLogs(
    const controller::ControlCommand *cmd,
    const controller::SimpleLateralDebug *debug_lat,
    const controller::SimpleLongitudinalDebug *debug_lon,
    const canbus::Chassis *chassis) {
  char buffer[2000];
  memset(buffer, 0, sizeof(buffer));
  snprintf(
      buffer, 1999,
      "%.6lf,%.6lf,%.6lf,%.6lf,%.6lf,%.6lf,%.6lf,%.6lf,%.6lf,%.6lf,%.6lf,%.6lf,%.6lf,%.6lf,%.6lf,%.6lf, \
    %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %d,\
    %.6f, %.6f \r\n",
      debug_lat->lateral_error(), debug_lat->ref_heading(),
      debug_lat->heading(), debug_lat->heading_error(),
      debug_lat->heading_error_rate(), debug_lat->lateral_error_rate(),
      debug_lat->curvature(), debug_lat->steer_angle(),
      debug_lat->steer_angle_feedforward(),
      debug_lat->steer_angle_lateral_contribution(),
      debug_lat->steer_angle_lateral_rate_contribution(),
      debug_lat->steer_angle_heading_contribution(),
      debug_lat->steer_angle_heading_rate_contribution(),
      debug_lat->steer_angle_feedback(), chassis->steering_percentage(),
      injector_->vehicle_state()->linear_velocity(),
      debug_lon->station_reference(), debug_lon->station_error(),
      debug_lon->station_error_limited(), debug_lon->preview_station_error(),
      debug_lon->speed_reference(), debug_lon->speed_error(),
      debug_lon->speed_controller_input_limited(),
      debug_lon->preview_speed_reference(), debug_lon->preview_speed_error(),
      debug_lon->preview_acceleration_reference(),
      debug_lon->acceleration_cmd_closeloop(), debug_lon->acceleration_cmd(),
      debug_lon->acceleration_lookup(), debug_lon->speed_lookup(),
      debug_lon->calibration_value(), debug_lon->throttle_cmd(),
      debug_lon->brake_cmd(), debug_lon->speed_controller_input_limited(),
      debug_lon->is_full_stop(), target_point_.path_point().x(),
      target_point_.path_point().y());

  const std::string log_str = std::string(buffer);
  if (FLAGS_enable_csv_debug) {
    lat_lon_file_ << log_str;
  }
  // LOG(INFO) << "Lat_Lon_Control_Detail: " << log_str;
}

bool LatLonController::LoadControlConf(
    const controller::ControlConf *control_conf) {
  if (!control_conf) {
    LOG(ERROR) << "[LatLonController] control_conf == nullptr";
    return false;
  }
  vehicle_param_ = vehicle::VehicleParam();

  ts_ = control_conf->lat_controller_conf().ts();
  if (ts_ <= 0.0) {
    LOG(ERROR) << "[LatLonController] Invalid control update interval.";
    return false;
  }
  cf_ = control_conf->lat_controller_conf().cf();
  cr_ = control_conf->lat_controller_conf().cr();
  preview_window_ = control_conf->lat_controller_conf().preview_window();
  lookahead_station_low_speed_ =
      control_conf->lat_controller_conf().lookahead_station();
  lookback_station_low_speed_ =
      control_conf->lat_controller_conf().lookback_station();
  lookahead_station_high_speed_ =
      control_conf->lat_controller_conf().lookahead_station_high_speed();
  lookback_station_high_speed_ =
      control_conf->lat_controller_conf().lookback_station_high_speed();
  wheelbase_ = vehicle_param_.wheel_base();
  steer_ratio_ = vehicle_param_.steer_ratio();
  steer_single_direction_max_degree_ =
      vehicle_param_.max_steer_angle() / M_PI * 180;
  max_lat_acc_ = control_conf->lat_controller_conf().max_lateral_acceleration();
  low_speed_bound_ = control_conf_->lon_controller_conf().switch_speed();
  low_speed_window_ =
      control_conf_->lon_controller_conf().switch_speed_window();

  const double mass_fl = control_conf->lat_controller_conf().mass_fl();
  const double mass_fr = control_conf->lat_controller_conf().mass_fr();
  const double mass_rl = control_conf->lat_controller_conf().mass_rl();
  const double mass_rr = control_conf->lat_controller_conf().mass_rr();
  const double mass_front = mass_fl + mass_fr;
  const double mass_rear = mass_rl + mass_rr;
  mass_ = mass_front + mass_rear;

  lf_ = wheelbase_ * (1.0 - mass_front / mass_);
  lr_ = wheelbase_ * (1.0 - mass_rear / mass_);

  // moment of inertia
  iz_ = lf_ * lf_ * mass_front + lr_ * lr_ * mass_rear;

  lqr_eps_ = control_conf->lat_controller_conf().eps();
  lqr_max_iteration_ = control_conf->lat_controller_conf().max_iteration();

  minimum_speed_protection_ = control_conf->minimum_speed_protection();

  return true;
}

void LatLonController::LogInitParameters() {
  LOG(INFO) << name_ << " begin.";
  LOG(INFO) << "[LatLonController parameters]"
            << " mass_: " << mass_ << ","
            << " iz_: " << iz_ << ","
            << " lf_: " << lf_ << ","
            << " lr_: " << lr_;
}

void LatLonController::InitializeFilters(
    const controller::ControlConf *control_conf) {
  // Low pass filter
  std::vector<double> den(3, 0.0);
  std::vector<double> num(3, 0.0);
  LpfCoefficients(ts_, control_conf->lat_controller_conf().cutoff_freq(), &den,
                  &num);
  digital_filter_.set_coefficients(den, num);
  lateral_error_filter_ = MeanFilter(static_cast<std::uint_fast8_t>(
      control_conf->lat_controller_conf().mean_filter_window_size()));
  heading_error_filter_ = MeanFilter(static_cast<std::uint_fast8_t>(
      control_conf->lat_controller_conf().mean_filter_window_size()));
}

void LatLonController::LoadLatGainScheduler(
    const controller::LatControllerConf &lat_controller_conf) {
  const auto &lat_err_gain_scheduler =
      lat_controller_conf.lat_err_gain_scheduler();
  const auto &heading_err_gain_scheduler =
      lat_controller_conf.heading_err_gain_scheduler();
  LOG(INFO) << "Lateral control gain scheduler loaded";
  Interpolation1D::DataType xy1, xy2;
  for (const auto &scheduler : lat_err_gain_scheduler.scheduler()) {
    xy1.push_back(std::make_pair(scheduler.speed(), scheduler.ratio()));
  }
  for (const auto &scheduler : heading_err_gain_scheduler.scheduler()) {
    xy2.push_back(std::make_pair(scheduler.speed(), scheduler.ratio()));
  }

  lat_err_interpolation_.reset(new Interpolation1D);
  CHECK(lat_err_interpolation_->Init(xy1))
      << "Fail to load lateral error gain scheduler";

  heading_err_interpolation_.reset(new Interpolation1D);
  CHECK(heading_err_interpolation_->Init(xy2))
      << "Fail to load heading error gain scheduler";
}

void LatLonController::Stop() { CloseLogFile(); }

std::string LatLonController::Name() const { return name_; }

Status LatLonController::LonReset() {
  speed_pid_controller_.Reset();
  station_pid_controller_.Reset();
  return Status::OK();
}

Status LatLonController::LatReset() {
  matrix_state_.setZero();
  digital_filter_.reset_values();
  heading_error_filter_.Reset();
  lateral_error_filter_.Reset();
  if (enable_mrac_) {
    mrac_controller_.Reset();
  }
  return Status::OK();
}

Status LatLonController::Reset() {
  LonReset();
  LatReset();
  reached_ = false;
  return Status::OK();
}

void LatLonController::GetTargetPoint(
    const TrajectoryAnalyzer *trajectory_analyzer, const double preview_time) {
  auto vehicle_state = injector_->vehicle_state();
  auto goal = trajectory_analyzer_->trajectory_points().back();

  matched_point_ = trajectory_analyzer->QueryMatchedPathPoint(
      vehicle_state->x(), vehicle_state->y());

  trajectory_analyzer->ToTrajectoryFrame(
      vehicle_state->x(), vehicle_state->y(), vehicle_state->heading(),
      vehicle_state->linear_velocity(), matched_point_, &s_matched_,
      &s_dot_matched_, &d_matched_, &d_dot_matched_);

  if (FLAGS_enable_control_by_time) {
    double current_control_time = Timer::Now();
    double preview_control_time = current_control_time + preview_time;

    reference_point_ = trajectory_analyzer->QueryNearestPointByAbsoluteTime(
        current_control_time);
    preview_point_ = trajectory_analyzer->QueryNearestPointByAbsoluteTime(
        preview_control_time);

  } else {
    if (FLAGS_enable_speed_station_preview) {
      size_t index_min = trajectory_analyzer->QueryNearestIndexByPosition(
          vehicle_state->x(), vehicle_state->y());
      reference_point_ = trajectory_analyzer->trajectory_points().at(index_min);
    }
    size_t target_index = trajectory_analyzer->QueryIndexByPosition(
        vehicle_state->x(), vehicle_state->y(), 0.1);
    preview_point_ = trajectory_analyzer->trajectory_points().at(target_index);
  }
  target_point_ =
      FLAGS_enable_speed_station_preview ? preview_point_ : reference_point_;
  double dx = goal.path_point().x() - vehicle_state->x();
  double dy = goal.path_point().y() - vehicle_state->y();
  bool distance_check = (dx * dx + dy * dy) < 0.0025;
  bool overtake_check =
      std::fabs(s_matched_) > std::fabs(goal.path_point().s());
  reached_ = distance_check || overtake_check;
}

void LatLonController::ComputeLongitudinalErrors(
    const double ts, controller::SimpleLongitudinalDebug *debug) {
  auto vehicle_state = injector_->vehicle_state();
  debug->mutable_current_matched_point()->mutable_path_point()->set_x(
      matched_point_.x());
  debug->mutable_current_matched_point()->mutable_path_point()->set_y(
      matched_point_.y());
  debug->mutable_current_reference_point()->mutable_path_point()->set_x(
      reference_point_.path_point().x());
  debug->mutable_current_reference_point()->mutable_path_point()->set_y(
      reference_point_.path_point().y());
  debug->mutable_preview_reference_point()->mutable_path_point()->set_x(
      preview_point_.path_point().x());
  debug->mutable_preview_reference_point()->mutable_path_point()->set_y(
      preview_point_.path_point().y());

  // LOG(INFO) << "matched point:" << matched_point_.DebugString();
  // LOG(INFO) << "reference point:" << reference_point_.DebugString();
  // LOG(INFO) << "preview point:" << preview_point_.DebugString();

  double heading_error =
      NormalizeAngle(vehicle_state->heading() - matched_point_.theta());
  double lon_speed = vehicle_state->linear_velocity() * std::cos(heading_error);
  double lon_acceleration =
      vehicle_state->linear_acceleration() * std::cos(heading_error);
  double one_minus_kappa_lat_error = 1 - reference_point_.path_point().kappa() *
                                             vehicle_state->linear_velocity() *
                                             std::sin(heading_error);

  debug->set_station_reference(reference_point_.path_point().s());
  debug->set_current_station(s_matched_);
  debug->set_station_error(reference_point_.path_point().s() - s_matched_);
  debug->set_speed_reference(reference_point_.v());
  debug->set_current_speed(lon_speed);
  debug->set_speed_error(reference_point_.v() - s_dot_matched_);
  debug->set_acceleration_reference(reference_point_.a());
  debug->set_current_acceleration(lon_acceleration);
  debug->set_acceleration_error(reference_point_.a() -
                                lon_acceleration / one_minus_kappa_lat_error);
  double jerk_reference =
      (debug->acceleration_reference() - previous_acceleration_reference_) / ts;
  double lon_jerk =
      (debug->current_acceleration() - previous_acceleration_) / ts;
  debug->set_jerk_reference(jerk_reference);
  debug->set_current_jerk(lon_jerk);
  debug->set_jerk_error(jerk_reference - lon_jerk / one_minus_kappa_lat_error);
  previous_acceleration_reference_ = debug->acceleration_reference();
  previous_acceleration_ = debug->current_acceleration();

  debug->set_preview_station_error(preview_point_.path_point().s() -
                                   s_matched_);
  debug->set_preview_speed_error(preview_point_.v() - s_dot_matched_);
  debug->set_preview_speed_reference(preview_point_.v());
  debug->set_preview_acceleration_reference(preview_point_.a());
}

void LatLonController::ComputeLateralErrors(
    const double x, const double y, const double theta, const double linear_v,
    const double angular_v, const double linear_a,
    const TrajectoryAnalyzer *trajectory_analyzer,
    controller::SimpleLateralDebug *debug) {
  const double dx = x - target_point_.path_point().x();
  const double dy = y - target_point_.path_point().y();

  debug->mutable_current_target_point()->mutable_path_point()->set_x(
      target_point_.path_point().x());
  debug->mutable_current_target_point()->mutable_path_point()->set_y(
      target_point_.path_point().y());

  // LOG(INFO) << "x point: " << x << " y point: " << y;
  // LOG(INFO) << "match point information : " <<
  // target_point_.ShortDebugString();

  const double cos_target_heading =
      std::cos(target_point_.path_point().theta());
  const double sin_target_heading =
      std::sin(target_point_.path_point().theta());

  double lateral_error = cos_target_heading * dy - sin_target_heading * dx;
  if (FLAGS_enable_navigation_mode_error_filter) {
    lateral_error = lateral_error_filter_.Update(lateral_error);
  }

  debug->set_lateral_error(lateral_error);

  debug->set_ref_heading(target_point_.path_point().theta());
  double heading_error = NormalizeAngle(theta - debug->ref_heading());
  if (FLAGS_enable_navigation_mode_error_filter) {
    heading_error = heading_error_filter_.Update(heading_error);
  }
  debug->set_heading_error(heading_error);

  // Within the low-high speed transition window, linerly interplolate the
  // lookahead/lookback station for "soft" prediction window switch
  double lookahead_station = 0.0;
  double lookback_station = 0.0;
  if (std::fabs(linear_v) >= low_speed_bound_) {
    lookahead_station = lookahead_station_high_speed_;
    lookback_station = lookback_station_high_speed_;
  } else if (std::fabs(linear_v) < low_speed_bound_ - low_speed_window_) {
    lookahead_station = lookahead_station_low_speed_;
    lookback_station = lookback_station_low_speed_;
  } else {
    lookahead_station = lerp(
        lookahead_station_low_speed_, low_speed_bound_ - low_speed_window_,
        lookahead_station_high_speed_, low_speed_bound_, std::fabs(linear_v));
    lookback_station = lerp(
        lookback_station_low_speed_, low_speed_bound_ - low_speed_window_,
        lookback_station_high_speed_, low_speed_bound_, std::fabs(linear_v));
  }

  // Estimate the heading error with look-ahead/look-back windows as feedback
  // signal for special driving scenarios
  double heading_error_feedback;
  if (injector_->vehicle_state()->gear() == canbus::Chassis::GEAR_REVERSE) {
    heading_error_feedback = heading_error;
  } else {
    auto lookahead_point = trajectory_analyzer->QueryNearestPointByRelativeTime(
        target_point_.relative_time() +
        lookahead_station /
            (std::max(std::fabs(linear_v), 0.1) * std::cos(heading_error)));
    heading_error_feedback =
        NormalizeAngle(heading_error + target_point_.path_point().theta() -
                       lookahead_point.path_point().theta());
  }
  debug->set_heading_error_feedback(heading_error_feedback);

  // Estimate the lateral error with look-ahead/look-back windows as feedback
  // signal for special driving scenarios
  double lateral_error_feedback;
  if (injector_->vehicle_state()->gear() == canbus::Chassis::GEAR_REVERSE) {
    lateral_error_feedback =
        lateral_error - lookback_station * std::sin(heading_error);
  } else {
    lateral_error_feedback =
        lateral_error + lookahead_station * std::sin(heading_error);
  }
  debug->set_lateral_error_feedback(lateral_error_feedback);

  auto lateral_error_dot = linear_v * std::sin(heading_error);
  auto lateral_error_dot_dot = linear_a * std::sin(heading_error);
  if (FLAGS_reverse_heading_control) {
    if (injector_->vehicle_state()->gear() == canbus::Chassis::GEAR_REVERSE) {
      lateral_error_dot = -lateral_error_dot;
      lateral_error_dot_dot = -lateral_error_dot_dot;
    }
  }
  debug->set_lateral_error_rate(lateral_error_dot);
  debug->set_lateral_acceleration(lateral_error_dot_dot);
  debug->set_lateral_jerk(
      (debug->lateral_acceleration() - previous_lateral_acceleration_) / ts_);
  previous_lateral_acceleration_ = debug->lateral_acceleration();

  if (injector_->vehicle_state()->gear() == canbus::Chassis::GEAR_REVERSE) {
    debug->set_heading_rate(-angular_v);
  } else {
    debug->set_heading_rate(angular_v);
  }
  debug->set_ref_heading_rate(target_point_.path_point().kappa() *
                              target_point_.v());
  debug->set_heading_error_rate(debug->heading_rate() -
                                debug->ref_heading_rate());

  debug->set_heading_acceleration(
      (debug->heading_rate() - previous_heading_rate_) / ts_);
  debug->set_ref_heading_acceleration(
      (debug->ref_heading_rate() - previous_ref_heading_rate_) / ts_);
  debug->set_heading_error_acceleration(debug->heading_acceleration() -
                                        debug->ref_heading_acceleration());
  previous_heading_rate_ = debug->heading_rate();
  previous_ref_heading_rate_ = debug->ref_heading_rate();

  debug->set_heading_jerk(
      (debug->heading_acceleration() - previous_heading_acceleration_) / ts_);
  debug->set_ref_heading_jerk(
      (debug->ref_heading_acceleration() - previous_ref_heading_acceleration_) /
      ts_);
  debug->set_heading_error_jerk(debug->heading_jerk() -
                                debug->ref_heading_jerk());
  previous_heading_acceleration_ = debug->heading_acceleration();
  previous_ref_heading_acceleration_ = debug->ref_heading_acceleration();

  debug->set_curvature(target_point_.path_point().kappa());
}

void LatLonController::UpdateState(controller::SimpleLateralDebug *debug) {
  auto vehicle_state = injector_->vehicle_state();
  if (FLAGS_use_navigation_mode) {
    ComputeLateralErrors(
        0.0, 0.0, driving_orientation_, vehicle_state->linear_velocity(),
        vehicle_state->angular_velocity(), vehicle_state->linear_acceleration(),
        trajectory_analyzer_.get(), debug);
  } else {
    // Transform the coordinate of the vehicle states from the center of the
    // rear-axis to the center of mass, if conditions matched
    const auto &com = vehicle_state->ComputeCOMPosition(lr_);
    ComputeLateralErrors(com.x(), com.y(), driving_orientation_,
                         vehicle_state->linear_velocity(),
                         vehicle_state->angular_velocity(),
                         vehicle_state->linear_acceleration(),
                         trajectory_analyzer_.get(), debug);
  }

  // State matrix update;
  // First four elements are fixed;
  if (enable_look_ahead_back_control_) {
    matrix_state_(0, 0) = debug->lateral_error_feedback();
    matrix_state_(2, 0) = debug->heading_error_feedback();
  } else {
    matrix_state_(0, 0) = debug->lateral_error();
    matrix_state_(2, 0) = debug->heading_error();
  }
  matrix_state_(1, 0) = debug->lateral_error_rate();
  matrix_state_(3, 0) = debug->heading_error_rate();

  // Next elements are depending on preview window size;
  for (int i = 0; i < preview_window_; ++i) {
    const double preview_time = ts_ * (i + 1);
    const auto preview_point =
        trajectory_analyzer_->QueryNearestPointByRelativeTime(preview_time);

    const auto matched_point =
        trajectory_analyzer_->QueryNearestPointByPosition(
            preview_point.path_point().x(), preview_point.path_point().y());

    const double dx =
        preview_point.path_point().x() - matched_point.path_point().x();
    const double dy =
        preview_point.path_point().y() - matched_point.path_point().y();

    const double cos_matched_theta =
        std::cos(matched_point.path_point().theta());
    const double sin_matched_theta =
        std::sin(matched_point.path_point().theta());
    const double preview_d_error =
        cos_matched_theta * dy - sin_matched_theta * dx;

    matrix_state_(basic_state_size_ + i, 0) = preview_d_error;
  }
}

Status LatLonController::ComputeControlCommand(
    const controller::LocalizationEstimate *localization,
    const canbus::Chassis *chassis,
    const planning::ADCTrajectory *planning_published_trajectory,
    controller::ControlCommand *cmd) {
  // Lon controlling begins
  localization_ = localization;
  chassis_ = chassis;
  trajectory_message_ = planning_published_trajectory;
  if (!control_interpolation_) {
    LOG(ERROR) << "Fail to initialize calibration table.";
    return Status(controller::ErrorCode::CONTROL_COMPUTE_ERROR,
                  "Fail to initialize calibration table.");
  }

  if (trajectory_analyzer_ == nullptr ||
      trajectory_analyzer_->seq_num() !=
          trajectory_message_->header().sequence_num()) {
    trajectory_analyzer_.reset(new TrajectoryAnalyzer(trajectory_message_));
  }

  const controller::LonControllerConf &lon_controller_conf =
      control_conf_->lon_controller_conf();

  auto debug_lon = cmd->mutable_debug()->mutable_simple_lon_debug();
  debug_lon->Clear();

  double brake_cmd = 0.0;
  double throttle_cmd = 0.0;
  double ts = lon_controller_conf.ts();
  double preview_time = lon_controller_conf.preview_window() * ts;
  bool enable_leadlag =
      lon_controller_conf.enable_reverse_leadlag_compensation();
  if (preview_time < 0.0) {
    const auto error_msg =
        "Preview time set as: " + std::to_string(preview_time) + " less than 0";
    LOG(ERROR) << error_msg;
    return Status(controller::ErrorCode::CONTROL_COMPUTE_ERROR, error_msg);
  }
  GetTargetPoint(trajectory_analyzer_.get(), preview_time);

  ComputeLongitudinalErrors(ts, debug_lon);

  double station_error_limit = lon_controller_conf.station_error_limit();
  double station_error_limited = 0.0;

  if (trajectory_message_->gear() == canbus::Chassis::GEAR_REVERSE) {
    station_pid_controller_.SetPID(
        lon_controller_conf.reverse_station_pid_conf());
    speed_pid_controller_.SetPID(lon_controller_conf.reverse_speed_pid_conf());
    if (enable_leadlag) {
      station_leadlag_controller_.SetLeadlag(
          lon_controller_conf.reverse_station_leadlag_conf());
      speed_leadlag_controller_.SetLeadlag(
          lon_controller_conf.reverse_speed_leadlag_conf());
    }
  } else if (injector_->vehicle_state()->linear_velocity() <=
             lon_controller_conf.switch_speed()) {
    speed_pid_controller_.SetPID(lon_controller_conf.low_speed_pid_conf());
  } else {
    speed_pid_controller_.SetPID(lon_controller_conf.high_speed_pid_conf());
  }

  if (FLAGS_enable_speed_station_preview) {
    station_error_limited = Clamp(debug_lon->preview_station_error(),
                                  -station_error_limit, station_error_limit);
    // update new target point
    if (debug_lon->preview_speed_reference() !=
        previous_preview_speed_reference_) {
      previous_preview_speed_reference_ = debug_lon->preview_speed_reference();
      // LonReset();
    }
  } else {
    station_error_limited = Clamp(debug_lon->station_error(),
                                  -station_error_limit, station_error_limit);
    // update new target point
    if (debug_lon->speed_reference() != previous_speed_reference_) {
      previous_speed_reference_ = debug_lon->speed_reference();
      // LonReset();
    }
  }

  double speed_offset =
      station_pid_controller_.Control(station_error_limited, ts);
  if (enable_leadlag) {
    speed_offset = station_leadlag_controller_.Control(speed_offset, ts);
  }

  double speed_controller_input = 0.0;
  double speed_controller_input_limit =
      lon_controller_conf.speed_controller_input_limit();
  double speed_controller_input_limited = 0.0;
  if (FLAGS_enable_speed_station_preview) {
    speed_controller_input = speed_offset + debug_lon->preview_speed_error();
  } else {
    speed_controller_input = speed_offset + debug_lon->speed_error();
  }
  speed_controller_input_limited =
      Clamp(speed_controller_input, -speed_controller_input_limit,
            speed_controller_input_limit);

  // keep the consistance of speed and direction
  if (planning_published_trajectory->gear() == canbus::Chassis::GEAR_REVERSE) {
    speed_controller_input_limited = speed_controller_input_limited <= 0
                                         ? speed_controller_input_limited
                                         : -0.01;
  } else {
    speed_controller_input_limited = speed_controller_input_limited >= 0
                                         ? speed_controller_input_limited
                                         : 0.01;
  }
  cmd->set_speed(speed_controller_input_limited);

  double acceleration_cmd_closeloop = 0.0;

  acceleration_cmd_closeloop =
      speed_pid_controller_.Control(speed_controller_input_limited, ts);
  debug_lon->set_pid_saturation_status(
      speed_pid_controller_.IntegratorSaturationStatus());
  if (enable_leadlag) {
    acceleration_cmd_closeloop =
        speed_leadlag_controller_.Control(acceleration_cmd_closeloop, ts);
    debug_lon->set_leadlag_saturation_status(
        speed_leadlag_controller_.InnerstateSaturationStatus());
  }

  double slope_offset_compensation = digital_filter_pitch_angle_.Filter(
      GRA_ACC * std::sin(injector_->vehicle_state()->pitch()));

  if (std::isnan(slope_offset_compensation)) {
    slope_offset_compensation = 0;
  }

  debug_lon->set_slope_offset_compensation(slope_offset_compensation);

  double acceleration_cmd =
      acceleration_cmd_closeloop + debug_lon->preview_acceleration_reference() +
      FLAGS_enable_slope_offset * debug_lon->slope_offset_compensation();
  debug_lon->set_is_full_stop(false);
  GetPathRemain(debug_lon);

  // At near-stop stage, replace the brake control command with the standstill
  // acceleration if the former is even softer than the latter
  if ((trajectory_message_->trajectory_type() ==
       planning::ADCTrajectory::NORMAL) &&
      ((std::fabs(debug_lon->preview_acceleration_reference()) <=
            control_conf_->max_acceleration_when_stopped() &&
        std::fabs(debug_lon->preview_speed_reference()) <=
            vehicle_param_.max_abs_speed_when_stopped()) ||
       std::abs(debug_lon->path_remain()) <
           control_conf_->max_path_remain_when_stopped())) {
    acceleration_cmd =
        (chassis->gear_location() == canbus::Chassis::GEAR_REVERSE)
            ? std::max(acceleration_cmd,
                       -lon_controller_conf.standstill_acceleration())
            : std::min(acceleration_cmd,
                       lon_controller_conf.standstill_acceleration());
    LOG(INFO) << "Stop location reached";
    debug_lon->set_is_full_stop(true);
  }

  double throttle_lowerbound =
      std::max(vehicle_param_.throttle_deadzone(),
               lon_controller_conf.throttle_minimum_action());
  double brake_lowerbound =
      std::max(vehicle_param_.brake_deadzone(),
               lon_controller_conf.brake_minimum_action());
  double calibration_value = 0.0;
  double acceleration_lookup =
      (chassis->gear_location() == canbus::Chassis::GEAR_REVERSE)
          ? -acceleration_cmd
          : acceleration_cmd;

  if (FLAGS_use_preview_speed_for_table) {
    calibration_value = control_interpolation_->Interpolate(std::make_pair(
        debug_lon->preview_speed_reference(), acceleration_lookup));
  } else {
    calibration_value = control_interpolation_->Interpolate(
        std::make_pair(chassis_->speed_mps(), acceleration_lookup));
  }

  if (acceleration_lookup >= 0) {
    if (calibration_value >= 0) {
      throttle_cmd = std::max(calibration_value, throttle_lowerbound);
    } else {
      throttle_cmd = throttle_lowerbound;
    }
    brake_cmd = 0.0;
  } else {
    throttle_cmd = 0.0;
    if (calibration_value >= 0) {
      brake_cmd = brake_lowerbound;
    } else {
      brake_cmd = std::max(-calibration_value, brake_lowerbound);
    }
  }

  debug_lon->set_station_error_limited(station_error_limited);
  debug_lon->set_speed_offset(speed_offset);
  debug_lon->set_speed_controller_input_limited(speed_controller_input_limited);
  debug_lon->set_acceleration_cmd(acceleration_cmd);
  debug_lon->set_throttle_cmd(throttle_cmd);
  debug_lon->set_brake_cmd(brake_cmd);
  debug_lon->set_acceleration_lookup(acceleration_lookup);
  debug_lon->set_speed_lookup(chassis_->speed_mps());
  debug_lon->set_calibration_value(calibration_value);
  debug_lon->set_acceleration_cmd_closeloop(acceleration_cmd_closeloop);

  // if the car is driven by acceleration, disgard the cmd->throttle and brake
  cmd->set_throttle(throttle_cmd);
  cmd->set_brake(brake_cmd);
  cmd->set_acceleration(acceleration_cmd);

  if (std::fabs(injector_->vehicle_state()->linear_velocity()) <=
          vehicle_param_.max_abs_speed_when_stopped() ||
      chassis->gear_location() == canbus::Chassis::GEAR_NEUTRAL) {
    cmd->set_gear_location(trajectory_message_->gear());
  } else {
    cmd->set_gear_location(chassis->gear_location());
  }
  // Lon Controlling ends

  // Lat Controlling begins
  auto vehicle_state = injector_->vehicle_state();

  auto target_tracking_trajectory = *planning_published_trajectory;
  trajectory_message_ = planning_published_trajectory;

  if (FLAGS_use_navigation_mode &&
      FLAGS_enable_navigation_mode_position_update) {
    auto time_stamp_diff =
        planning_published_trajectory->header().timestamp_sec() -
        current_trajectory_timestamp_;

    auto curr_vehicle_x = localization->pose().position().x();
    auto curr_vehicle_y = localization->pose().position().y();

    double curr_vehicle_heading = 0.0;
    const auto &orientation = localization->pose().orientation();
    if (localization->pose().has_heading()) {
      curr_vehicle_heading = localization->pose().heading();
    } else {
      curr_vehicle_heading =
          QuaternionToHeading(orientation.qw(), orientation.qx(),
                              orientation.qy(), orientation.qz());
    }

    // new planning trajectory
    if (time_stamp_diff > 1.0e-6) {
      init_vehicle_x_ = curr_vehicle_x;
      init_vehicle_y_ = curr_vehicle_y;
      init_vehicle_heading_ = curr_vehicle_heading;

      current_trajectory_timestamp_ =
          planning_published_trajectory->header().timestamp_sec();
    } else {
      auto x_diff_map = curr_vehicle_x - init_vehicle_x_;
      auto y_diff_map = curr_vehicle_y - init_vehicle_y_;
      auto theta_diff = curr_vehicle_heading - init_vehicle_heading_;

      auto cos_map_veh = std::cos(init_vehicle_heading_);
      auto sin_map_veh = std::sin(init_vehicle_heading_);

      auto x_diff_veh = cos_map_veh * x_diff_map + sin_map_veh * y_diff_map;
      auto y_diff_veh = -sin_map_veh * x_diff_map + cos_map_veh * y_diff_map;

      auto cos_theta_diff = std::cos(-theta_diff);
      auto sin_theta_diff = std::sin(-theta_diff);

      auto tx = -(cos_theta_diff * x_diff_veh - sin_theta_diff * y_diff_veh);
      auto ty = -(sin_theta_diff * x_diff_veh + cos_theta_diff * y_diff_veh);

      auto ptr_trajectory_points =
          target_tracking_trajectory.mutable_trajectory_point();
      std::for_each(ptr_trajectory_points->begin(),
                    ptr_trajectory_points->end(),
                    [&cos_theta_diff, &sin_theta_diff, &tx, &ty,
                     &theta_diff](TrajectoryPoint &p) {
                      auto x = p.path_point().x();
                      auto y = p.path_point().y();
                      auto theta = p.path_point().theta();

                      auto x_new = cos_theta_diff * x - sin_theta_diff * y + tx;
                      auto y_new = sin_theta_diff * x + cos_theta_diff * y + ty;
                      auto theta_new = NormalizeAngle(theta - theta_diff);

                      p.mutable_path_point()->set_x(x_new);
                      p.mutable_path_point()->set_y(y_new);
                      p.mutable_path_point()->set_theta(theta_new);
                    });
    }
  }

  // Transform the coordinate of the planning trajectory from the center of the
  // rear-axis to the center of mass, if conditions matched
  if (((FLAGS_trajectory_transform_to_com_reverse &&
        vehicle_state->gear() == canbus::Chassis::GEAR_REVERSE) ||
       (FLAGS_trajectory_transform_to_com_drive &&
        vehicle_state->gear() == canbus::Chassis::GEAR_DRIVE)) &&
      enable_look_ahead_back_control_) {
    trajectory_analyzer_->TrajectoryTransformToCOM(lr_);
  }

  // Re-build the vehicle dynamic models at reverse driving (in particular,
  // replace the lateral translational motion dynamics with the corresponding
  // kinematic models)
  if (vehicle_state->gear() == canbus::Chassis::GEAR_REVERSE) {
    /*
    A matrix (Gear Reverse)
    [0.0, 0.0, 1.0 * v 0.0;
     0.0, (-(c_f + c_r) / m) / v, (c_f + c_r) / m,
     (l_r * c_r - l_f * c_f) / m / v;
     0.0, 0.0, 0.0, 1.0;
     0.0, ((lr * cr - lf * cf) / i_z) / v, (l_f * c_f - l_r * c_r) / i_z,
     (-1.0 * (l_f^2 * c_f + l_r^2 * c_r) / i_z) / v;]
    */
    cf_ = -control_conf_->lat_controller_conf().cf();
    cr_ = -control_conf_->lat_controller_conf().cr();
    matrix_a_(0, 1) = 0.0;
    matrix_a_coeff_(0, 2) = 1.0;
  } else {
    /*
    A matrix (Gear Drive)
    [0.0, 1.0, 0.0, 0.0;
     0.0, (-(c_f + c_r) / m) / v, (c_f + c_r) / m,
     (l_r * c_r - l_f * c_f) / m / v;
     0.0, 0.0, 0.0, 1.0;
     0.0, ((lr * cr - lf * cf) / i_z) / v, (l_f * c_f - l_r * c_r) / i_z,
     (-1.0 * (l_f^2 * c_f + l_r^2 * c_r) / i_z) / v;]
    */
    cf_ = control_conf_->lat_controller_conf().cf();
    cr_ = control_conf_->lat_controller_conf().cr();
    matrix_a_(0, 1) = 1.0;
    matrix_a_coeff_(0, 2) = 0.0;
  }
  matrix_a_(1, 2) = (cf_ + cr_) / mass_;
  matrix_a_(3, 2) = (lf_ * cf_ - lr_ * cr_) / iz_;
  matrix_a_coeff_(1, 1) = -(cf_ + cr_) / mass_;
  matrix_a_coeff_(1, 3) = (lr_ * cr_ - lf_ * cf_) / mass_;
  matrix_a_coeff_(3, 1) = (lr_ * cr_ - lf_ * cf_) / iz_;
  matrix_a_coeff_(3, 3) = -1.0 * (lf_ * lf_ * cf_ + lr_ * lr_ * cr_) / iz_;

  /*
  b = [0.0, c_f / m, 0.0, l_f * c_f / i_z]^T
  */
  matrix_b_(1, 0) = cf_ / mass_;
  matrix_b_(3, 0) = lf_ * cf_ / iz_;
  matrix_bd_ = matrix_b_ * ts_;

  UpdateDrivingOrientation();

  controller::SimpleLateralDebug *debug_lat =
      cmd->mutable_debug()->mutable_simple_lat_debug();
  debug_lat->Clear();

  // Update state = [Lateral Error, Lateral Error Rate, Heading Error, Heading
  // Error Rate, preview lateral error1 , preview lateral error2, ...]
  UpdateState(debug_lat);

  UpdateMatrix();

  // Compound discrete matrix with road preview model
  UpdateMatrixCompound();

  // Adjust matrix_q_updated when in reverse gear
  int q_param_size = control_conf_->lat_controller_conf().matrix_q_size();
  int reverse_q_param_size =
      control_conf_->lat_controller_conf().reverse_matrix_q_size();
  if (injector_->vehicle_state()->gear() == canbus::Chassis::GEAR_REVERSE) {
    for (int i = 0; i < reverse_q_param_size; ++i) {
      matrix_q_(i, i) =
          control_conf_->lat_controller_conf().reverse_matrix_q(i);
    }
  } else {
    for (int i = 0; i < q_param_size; ++i) {
      matrix_q_(i, i) = control_conf_->lat_controller_conf().matrix_q(i);
    }
  }

  // Add gain scheduler for higher speed steering
  if (FLAGS_enable_gain_scheduler) {
    matrix_q_updated_(0, 0) =
        matrix_q_(0, 0) * lat_err_interpolation_->Interpolate(
                              std::fabs(vehicle_state->linear_velocity()));
    matrix_q_updated_(2, 2) =
        matrix_q_(2, 2) * heading_err_interpolation_->Interpolate(
                              std::fabs(vehicle_state->linear_velocity()));
    SolveLQRProblem(matrix_adc_, matrix_bdc_, matrix_q_updated_, matrix_r_,
                    lqr_eps_, lqr_max_iteration_, &matrix_k_);
  } else {
    SolveLQRProblem(matrix_adc_, matrix_bdc_, matrix_q_, matrix_r_, lqr_eps_,
                    lqr_max_iteration_, &matrix_k_);
  }

  // feedback = - K * state
  // Convert vehicle steer angle from rad to degree and then to steer degree
  // then to 100% ratio
  const double steer_angle_feedback = -(matrix_k_ * matrix_state_)(0, 0) * 180 /
                                      M_PI * steer_ratio_ /
                                      steer_single_direction_max_degree_ * 100;

  const double steer_angle_feedforward =
      ComputeFeedForward(debug_lat->curvature());

  double steer_angle = 0.0;
  double steer_angle_feedback_augment = 0.0;
  // Augment the feedback control on lateral error at the desired speed domain
  if (enable_leadlag_) {
    if (FLAGS_enable_feedback_augment_on_high_speed ||
        std::fabs(vehicle_state->linear_velocity()) < low_speed_bound_) {
      steer_angle_feedback_augment =
          leadlag_controller_.Control(-matrix_state_(0, 0), ts_) * 180 / M_PI *
          steer_ratio_ / steer_single_direction_max_degree_ * 100;
      if (std::fabs(vehicle_state->linear_velocity()) >
          low_speed_bound_ - low_speed_window_) {
        // Within the low-high speed transition window, linerly interplolate the
        // augment control gain for "soft" control switch
        steer_angle_feedback_augment = lerp(
            steer_angle_feedback_augment, low_speed_bound_ - low_speed_window_,
            0.0, low_speed_bound_, std::fabs(vehicle_state->linear_velocity()));
      }
    }
  }
  steer_angle = steer_angle_feedback + steer_angle_feedforward +
                steer_angle_feedback_augment;

  // Compute the steering command limit with the given maximum lateral
  // acceleration
  const double steer_limit =
      FLAGS_set_steer_limit ? std::atan(max_lat_acc_ * wheelbase_ /
                                        (vehicle_state->linear_velocity() *
                                         vehicle_state->linear_velocity())) *
                                  steer_ratio_ * 180 / M_PI /
                                  steer_single_direction_max_degree_ * 100
                            : 100.0;

  const double steer_diff_with_max_rate =
      FLAGS_enable_maximum_steer_rate_limit
          ? vehicle_param_.max_steer_angle_rate() * ts_ * 180 / M_PI /
                steer_single_direction_max_degree_ * 100
          : 100.0;

  const double steering_position = chassis->steering_percentage();

  // Re-compute the steering command if the MRAC control is enabled, with steer
  // angle limitation and steer rate limitation
  if (enable_mrac_) {
    const int mrac_model_order = control_conf_->lat_controller_conf()
                                     .steer_mrac_conf()
                                     .mrac_model_order();
    Matrix steer_state = Matrix::Zero(mrac_model_order, 1);
    steer_state(0, 0) = chassis->steering_percentage();
    if (mrac_model_order > 1) {
      steer_state(1, 0) = (steering_position - pre_steering_position_) / ts_;
    }
    if (std::fabs(vehicle_state->linear_velocity()) >
        control_conf_->minimum_speed_resolution()) {
      mrac_controller_.SetStateAdaptionRate(1.0);
      mrac_controller_.SetInputAdaptionRate(1.0);
    } else {
      mrac_controller_.SetStateAdaptionRate(0.0);
      mrac_controller_.SetInputAdaptionRate(0.0);
    }
    steer_angle = mrac_controller_.Control(
        steer_angle, steer_state, steer_limit, steer_diff_with_max_rate / ts_);
    // Set the steer mrac debug message
    controller::MracDebug *mracdebug = debug_lat->mutable_steer_mrac_debug();
    Matrix steer_reference = mrac_controller_.CurrentReferenceState();
    mracdebug->set_mrac_model_order(mrac_model_order);
    for (int i = 0; i < mrac_model_order; ++i) {
      mracdebug->add_mrac_reference_state(steer_reference(i, 0));
      mracdebug->add_mrac_state_error(steer_state(i, 0) -
                                      steer_reference(i, 0));
      mracdebug->mutable_mrac_adaptive_gain()->add_state_adaptive_gain(
          mrac_controller_.CurrentStateAdaptionGain()(i, 0));
    }
    mracdebug->mutable_mrac_adaptive_gain()->add_input_adaptive_gain(
        mrac_controller_.CurrentInputAdaptionGain()(0, 0));
    mracdebug->set_mrac_reference_saturation_status(
        mrac_controller_.ReferenceSaturationStatus());
    mracdebug->set_mrac_control_saturation_status(
        mrac_controller_.ControlSaturationStatus());
  }
  pre_steering_position_ = steering_position;
  debug_lat->set_steer_mrac_enable_status(enable_mrac_);

  // Clamp the steer angle with steer limitations at current speed
  double steer_angle_limited = Clamp(steer_angle, -steer_limit, steer_limit);
  steer_angle = steer_angle_limited;
  debug_lat->set_steer_angle_limited(steer_angle_limited);

  // Limit the steering command with the designed digital filter
  steer_angle = digital_filter_.Filter(steer_angle);
  steer_angle = Clamp(steer_angle, -100.0, 100.0);

  // Check if the steer is locked and hence the previous steer angle should be
  // executed
  if (std::abs(vehicle_state->linear_velocity()) < FLAGS_lock_steer_speed &&
      (vehicle_state->gear() == canbus::Chassis::GEAR_DRIVE ||
       vehicle_state->gear() == canbus::Chassis::GEAR_REVERSE) &&
      chassis->driving_mode() == canbus::Chassis::COMPLETE_AUTO_DRIVE) {
    steer_angle = pre_steer_angle_;
  }

  // Set the steer commands
  cmd->set_steering_target(Clamp(steer_angle,
                                 pre_steer_angle_ - steer_diff_with_max_rate,
                                 pre_steer_angle_ + steer_diff_with_max_rate));
  cmd->set_steering_rate(FLAGS_steer_angle_rate);

  pre_steer_angle_ = cmd->steering_target();

  // compute extra information for logging and debugging
  const double steer_angle_lateral_contribution =
      -matrix_k_(0, 0) * matrix_state_(0, 0) * 180 / M_PI * steer_ratio_ /
      steer_single_direction_max_degree_ * 100;

  const double steer_angle_lateral_rate_contribution =
      -matrix_k_(0, 1) * matrix_state_(1, 0) * 180 / M_PI * steer_ratio_ /
      steer_single_direction_max_degree_ * 100;

  const double steer_angle_heading_contribution =
      -matrix_k_(0, 2) * matrix_state_(2, 0) * 180 / M_PI * steer_ratio_ /
      steer_single_direction_max_degree_ * 100;

  const double steer_angle_heading_rate_contribution =
      -matrix_k_(0, 3) * matrix_state_(3, 0) * 180 / M_PI * steer_ratio_ /
      steer_single_direction_max_degree_ * 100;

  debug_lat->set_heading(driving_orientation_);
  debug_lat->set_steer_angle(steer_angle);
  debug_lat->set_steer_angle_feedforward(steer_angle_feedforward);
  debug_lat->set_steer_angle_lateral_contribution(
      steer_angle_lateral_contribution);
  debug_lat->set_steer_angle_lateral_rate_contribution(
      steer_angle_lateral_rate_contribution);
  debug_lat->set_steer_angle_heading_contribution(
      steer_angle_heading_contribution);
  debug_lat->set_steer_angle_heading_rate_contribution(
      steer_angle_heading_rate_contribution);
  debug_lat->set_steer_angle_feedback(steer_angle_feedback);
  debug_lat->set_steer_angle_feedback_augment(steer_angle_feedback_augment);
  debug_lat->set_steering_position(steering_position);
  debug_lat->set_ref_speed(vehicle_state->linear_velocity());
  // Lat controlling ends

  ProcessLogs(cmd, debug_lat, debug_lon, chassis_);
  return Status::OK();
}

void LatLonController::UpdateMatrix() {
  double v;
  // At reverse driving, replace the lateral translational motion dynamics with
  // the corresponding kinematic models
  if (injector_->vehicle_state()->gear() == canbus::Chassis::GEAR_REVERSE) {
    v = std::min(injector_->vehicle_state()->linear_velocity(),
                 -minimum_speed_protection_);
    matrix_a_(0, 2) = matrix_a_coeff_(0, 2) * v;
  } else {
    v = std::max(injector_->vehicle_state()->linear_velocity(),
                 minimum_speed_protection_);
    matrix_a_(0, 2) = 0.0;
  }
  matrix_a_(1, 1) = matrix_a_coeff_(1, 1) / v;
  matrix_a_(1, 3) = matrix_a_coeff_(1, 3) / v;
  matrix_a_(3, 1) = matrix_a_coeff_(3, 1) / v;
  matrix_a_(3, 3) = matrix_a_coeff_(3, 3) / v;
  Matrix matrix_i = Matrix::Identity(matrix_a_.cols(), matrix_a_.cols());
  matrix_ad_ = (matrix_i - ts_ * 0.5 * matrix_a_).inverse() *
               (matrix_i + ts_ * 0.5 * matrix_a_);
}

void LatLonController::UpdateMatrixCompound() {
  // Initialize preview matrix
  matrix_adc_.block(0, 0, basic_state_size_, basic_state_size_) = matrix_ad_;
  matrix_bdc_.block(0, 0, basic_state_size_, 1) = matrix_bd_;
  if (preview_window_ > 0) {
    matrix_bdc_(matrix_bdc_.rows() - 1, 0) = 1;
    // Update A matrix;
    for (int i = 0; i < preview_window_ - 1; ++i) {
      matrix_adc_(basic_state_size_ + i, basic_state_size_ + 1 + i) = 1;
    }
  }
}

double LatLonController::ComputeFeedForward(double ref_curvature) const {
  const double kv =
      lr_ * mass_ / 2 / cf_ / wheelbase_ - lf_ * mass_ / 2 / cr_ / wheelbase_;

  // Calculate the feedforward term of the lateral controller; then change it
  // from rad to %
  const double v = injector_->vehicle_state()->linear_velocity();
  double steer_angle_feedforwardterm;
  if (injector_->vehicle_state()->gear() == canbus::Chassis::GEAR_REVERSE) {
    steer_angle_feedforwardterm = std::atan(wheelbase_ * ref_curvature) * 180 /
                                  M_PI * steer_ratio_ /
                                  steer_single_direction_max_degree_ * 100;
  } else {
    steer_angle_feedforwardterm =
        (wheelbase_ * ref_curvature + kv * v * v * ref_curvature -
         matrix_k_(0, 2) *
             (lr_ * ref_curvature -
              lf_ * mass_ * v * v * ref_curvature / 2.0 / cr_ / wheelbase_)) *
        180 / M_PI * steer_ratio_ / steer_single_direction_max_degree_ * 100;
  }

  return steer_angle_feedforwardterm;
}

void LatLonController::UpdateDrivingOrientation() {
  auto vehicle_state = injector_->vehicle_state();
  driving_orientation_ = vehicle_state->heading();
  matrix_bd_ = matrix_b_ * ts_;
  // Reverse the driving direction if the vehicle is in reverse mode
  if (FLAGS_reverse_heading_control) {
    if (vehicle_state->gear() == canbus::Chassis::GEAR_REVERSE) {
      driving_orientation_ = NormalizeAngle(driving_orientation_ + M_PI);
      // Update Matrix_b for reverse mode
      matrix_bd_ = -matrix_b_ * ts_;
      LOG(INFO) << "Matrix_b changed due to gear direction";
    }
  }
}

void LatLonController::SetDigitalFilter(double ts, double cutoff_freq,
                                        DigitalFilter *digital_filter) {
  std::vector<double> denominators;
  std::vector<double> numerators;
  LpfCoefficients(ts, cutoff_freq, &denominators, &numerators);
  digital_filter->set_coefficients(denominators, numerators);
}

// TODO(all): Refactor and simplify
void LatLonController::GetPathRemain(
    controller::SimpleLongitudinalDebug *debug) {
  int stop_index = 0;
  static constexpr double kSpeedThreshold = 1e-3;
  static constexpr double kForwardAccThreshold = -1e-2;
  static constexpr double kBackwardAccThreshold = 1e-1;
  static constexpr double kParkingSpeed = 0.1;

  if (trajectory_message_->gear() == canbus::Chassis::GEAR_DRIVE) {
    while (stop_index < trajectory_message_->trajectory_point_size()) {
      auto &current_trajectory_point =
          trajectory_message_->trajectory_point(stop_index);
      if (fabs(current_trajectory_point.v()) < kSpeedThreshold &&
          current_trajectory_point.a() > kForwardAccThreshold &&
          current_trajectory_point.a() < 0.0) {
        break;
      }
      ++stop_index;
    }
  } else {
    while (stop_index < trajectory_message_->trajectory_point_size()) {
      auto &current_trajectory_point =
          trajectory_message_->trajectory_point(stop_index);
      if (current_trajectory_point.v() < kSpeedThreshold &&
          current_trajectory_point.a() < kBackwardAccThreshold &&
          current_trajectory_point.a() > 0.0) {
        break;
      }
      ++stop_index;
    }
  }
  if (stop_index == trajectory_message_->trajectory_point_size()) {
    --stop_index;
    if (fabs(trajectory_message_->trajectory_point(stop_index).v()) <
        kParkingSpeed) {
      LOG(INFO) << "the last point is selected as parking point";
    } else {
      LOG(INFO) << "the last point found in path and speed > speed_deadzone";
    }
  }
  debug->set_path_remain(
      trajectory_message_->trajectory_point(stop_index).path_point().s() -
      debug->current_station());
}

void LatLonController::SetDigitalFilterPitchAngle(
    const controller::LonControllerConf &lon_controller_conf) {
  double cutoff_freq =
      lon_controller_conf.pitch_angle_filter_conf().cutoff_freq();
  double ts = lon_controller_conf.ts();
  SetDigitalFilter(ts, cutoff_freq, &digital_filter_pitch_angle_);
}

void LatLonController::LoadControlCalibrationTable(
    const controller::LonControllerConf &lon_controller_conf) {
  const auto &control_table = lon_controller_conf.calibration_table();
  // LOG(INFO) << "Control calibration table loaded";
  // LOG(INFO) << "Control calibration table size is "
  // << control_table.calibration_size();
  Interpolation2D::DataType xyz;
  for (const auto &calibration : control_table.calibration()) {
    xyz.push_back(std::make_tuple(calibration.speed(),
                                  calibration.acceleration(),
                                  calibration.command()));
  }
  control_interpolation_.reset(new Interpolation2D);
  CHECK(control_interpolation_->Init(xyz))
      << "Fail to load control calibration table";
}

Status LatLonController::Init(std::shared_ptr<DependencyInjector> injector,
                              const controller::ControlConf *control_conf) {
  control_conf_ = control_conf;
  if (control_conf_ == nullptr) {
    controller_initialized_ = false;
    LOG(ERROR) << "get_longitudinal_param() nullptr";
    return Status(controller::ErrorCode::CONTROL_INIT_ERROR,
                  "Failed to load LatLonController conf");
  }
  injector_ = injector;

  // LatLonController begins to initialize
  const controller::LonControllerConf &lon_controller_conf =
      control_conf_->lon_controller_conf();
  double ts = lon_controller_conf.ts();
  bool enable_leadlag =
      lon_controller_conf.enable_reverse_leadlag_compensation();

  station_pid_controller_.Init(lon_controller_conf.station_pid_conf());
  speed_pid_controller_.Init(lon_controller_conf.low_speed_pid_conf());

  if (enable_leadlag) {
    station_leadlag_controller_.Init(
        lon_controller_conf.reverse_station_leadlag_conf(), ts);
    speed_leadlag_controller_.Init(
        lon_controller_conf.reverse_speed_leadlag_conf(), ts);
  }

  vehicle_param_ = vehicle::VehicleParam();

  SetDigitalFilterPitchAngle(lon_controller_conf);

  LoadControlCalibrationTable(lon_controller_conf);
  controller_initialized_ = true;
  // LatLonController initialisation ends

  // LatLonController begins to initialize
  if (!LoadControlConf(control_conf_)) {
    LOG(ERROR) << "failed to load control conf";
    return Status(ErrorCode::CONTROL_COMPUTE_ERROR,
                  "failed to load control_conf");
  }
  // Matrix init operations.
  const int matrix_size = basic_state_size_ + preview_window_;
  matrix_a_ = Matrix::Zero(basic_state_size_, basic_state_size_);
  matrix_ad_ = Matrix::Zero(basic_state_size_, basic_state_size_);
  matrix_adc_ = Matrix::Zero(matrix_size, matrix_size);
  /*
  A matrix (Gear Drive)
  [0.0, 1.0, 0.0, 0.0;
   0.0, (-(c_f + c_r) / m) / v, (c_f + c_r) / m,
   (l_r * c_r - l_f * c_f) / m / v;
   0.0, 0.0, 0.0, 1.0;
   0.0, ((lr * cr - lf * cf) / i_z) / v, (l_f * c_f - l_r * c_r) / i_z,
   (-1.0 * (l_f^2 * c_f + l_r^2 * c_r) / i_z) / v;]
  */
  matrix_a_(0, 1) = 1.0;
  matrix_a_(1, 2) = (cf_ + cr_) / mass_;
  matrix_a_(2, 3) = 1.0;
  matrix_a_(3, 2) = (lf_ * cf_ - lr_ * cr_) / iz_;

  matrix_a_coeff_ = Matrix::Zero(matrix_size, matrix_size);
  matrix_a_coeff_(1, 1) = -(cf_ + cr_) / mass_;
  matrix_a_coeff_(1, 3) = (lr_ * cr_ - lf_ * cf_) / mass_;
  matrix_a_coeff_(3, 1) = (lr_ * cr_ - lf_ * cf_) / iz_;
  matrix_a_coeff_(3, 3) = -1.0 * (lf_ * lf_ * cf_ + lr_ * lr_ * cr_) / iz_;

  /*
  b = [0.0, c_f / m, 0.0, l_f * c_f / i_z]^T
  */
  matrix_b_ = Matrix::Zero(basic_state_size_, 1);
  matrix_bd_ = Matrix::Zero(basic_state_size_, 1);
  matrix_bdc_ = Matrix::Zero(matrix_size, 1);
  matrix_b_(1, 0) = cf_ / mass_;
  matrix_b_(3, 0) = lf_ * cf_ / iz_;
  matrix_bd_ = matrix_b_ * ts_;

  matrix_state_ = Matrix::Zero(matrix_size, 1);
  matrix_k_ = Matrix::Zero(1, matrix_size);
  matrix_r_ =
      Matrix::Identity(1, 1) * control_conf_->lat_controller_conf().matrix_r();
  matrix_q_ = Matrix::Zero(matrix_size, matrix_size);

  int q_param_size = control_conf_->lat_controller_conf().matrix_q_size();
  int reverse_q_param_size =
      control_conf_->lat_controller_conf().reverse_matrix_q_size();
  if (matrix_size != q_param_size || matrix_size != reverse_q_param_size) {
    const auto error_msg = "lateral controller error: matrix_q size: " +
                           std::to_string(q_param_size) +
                           "lateral controller error: reverse_matrix_q size: " +
                           std::to_string(reverse_q_param_size) +
                           " in parameter file not equal to matrix_size: " +
                           std::to_string(matrix_size);
    LOG(ERROR) << error_msg;
    return Status(ErrorCode::CONTROL_COMPUTE_ERROR, error_msg);
  }

  for (int i = 0; i < q_param_size; ++i) {
    matrix_q_(i, i) = control_conf_->lat_controller_conf().matrix_q(i);
  }

  matrix_q_updated_ = matrix_q_;
  InitializeFilters(control_conf_);
  auto &lat_controller_conf = control_conf_->lat_controller_conf();
  LoadLatGainScheduler(lat_controller_conf);
  LogInitParameters();

  enable_leadlag_ = control_conf_->lat_controller_conf()
                        .enable_reverse_leadlag_compensation();
  if (enable_leadlag_) {
    leadlag_controller_.Init(lat_controller_conf.reverse_leadlag_conf(), ts_);
  }

  enable_mrac_ =
      control_conf_->lat_controller_conf().enable_steer_mrac_control();
  if (enable_mrac_) {
    mrac_controller_.Init(lat_controller_conf.steer_mrac_conf(),
                          vehicle_param_.steering_latency_param(), ts_);
  }

  enable_look_ahead_back_control_ =
      control_conf_->lat_controller_conf().enable_look_ahead_back_control();

  // LatLonController initialisation ends

  return Status::OK();
}
