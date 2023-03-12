#include "pid_controller.h"

#include <cmath>

#include "glog/logging.h"

double PIDController::Control(const double error, const double dt) {
  if (dt <= 0) {
    LOG(INFO) << "dt <= 0, will use the last output, dt: " << dt;
    return previous_output_;
  }
  double diff = 0;
  double output = 0;

  if (first_hit_) {
    first_hit_ = false;
  } else {
    diff = (error - previous_error_) / dt;
  }
  // integral hold
  if (!integrator_enabled_) {
    integral_ = 0;
  } else if (!integrator_hold_) {
    integral_ += error * dt * ki_;
    // apply Ki before integrating to avoid steps when change Ki at steady state
    if (integral_ > integrator_saturation_high_) {
      integral_ = integrator_saturation_high_;
      integrator_saturation_status_ = 1;
    } else if (integral_ < integrator_saturation_low_) {
      integral_ = integrator_saturation_low_;
      integrator_saturation_status_ = -1;
    } else {
      integrator_saturation_status_ = 0;
    }
  }
  previous_error_ = error;
  output = error * kp_ + integral_ + diff * kd_;  // Ki already applied
  previous_output_ = output;
  return output;
}

void PIDController::Reset() {
  previous_error_ = 0.0;
  previous_output_ = 0.0;
  integral_ = 0.0;
  first_hit_ = true;
  integrator_saturation_status_ = 0;
  output_saturation_status_ = 0;
}

void PIDController::Init(const controller::PidConf &pid_conf) {
  previous_error_ = 0.0;
  previous_output_ = 0.0;
  integral_ = 0.0;
  first_hit_ = true;
  integrator_enabled_ = pid_conf.integrator_enable();
  integrator_saturation_high_ =
      std::fabs(pid_conf.integrator_saturation_level());
  integrator_saturation_low_ =
      -std::fabs(pid_conf.integrator_saturation_level());
  integrator_saturation_status_ = 0;
  integrator_hold_ = false;
  output_saturation_high_ = std::fabs(pid_conf.output_saturation_level());
  output_saturation_low_ = -std::fabs(pid_conf.output_saturation_level());
  output_saturation_status_ = 0;
  SetPID(pid_conf);
}

void PIDController::SetPID(const controller::PidConf &pid_conf) {
  kp_ = pid_conf.kp();
  ki_ = pid_conf.ki();
  kd_ = pid_conf.kd();
  kaw_ = pid_conf.kaw();
}

int PIDController::IntegratorSaturationStatus() const {
  return integrator_saturation_status_;
}

bool PIDController::IntegratorHold() const { return integrator_hold_; }

void PIDController::SetIntegratorHold(bool hold) { integrator_hold_ = hold; }
