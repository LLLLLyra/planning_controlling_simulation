#include "leadlag_controller.h"

#include <cmath>

#include "glog/logging.h"

double LeadlagController::Control(const double error, const double dt) {
  // check if the c2d transform passed during the initilization
  if (!transfromc2d_enabled_) {
    TransformC2d(dt);
    if (!transfromc2d_enabled_) {
      LOG(INFO)
          << "C2d transform failed; will work as a unity compensator, dt: "
          << dt;
      return error;  // treat the Lead/Lag as a unity proportional controller
    }
  }
  // check if the current sampling time is valid
  if (dt <= 0.0) {
    LOG(INFO) << "dt <= 0, will use the last output, dt: " << dt;
    return previous_output_;
  }
  double output = 0.0;

  innerstate_ = (error - previous_innerstate_ * kd0_) / kd1_;  // calculate
  // the inner (intermedia) state under the Direct form II for the Lead / Lag
  // compensator factorization
  if (innerstate_ > innerstate_saturation_high_) {
    innerstate_ = innerstate_saturation_high_;
    innerstate_saturation_status_ = 1;
  } else if (innerstate_ < innerstate_saturation_low_) {
    innerstate_ = innerstate_saturation_low_;
    innerstate_saturation_status_ = -1;
  } else {
    innerstate_saturation_status_ = 0;
  }

  output = innerstate_ * kn1_ + previous_innerstate_ * kn0_;
  previous_innerstate_ = innerstate_;
  previous_output_ = output;
  return output;
}

void LeadlagController::Reset() {
  previous_output_ = 0.0;
  previous_innerstate_ = 0.0;
  innerstate_ = 0.0;
  innerstate_saturation_status_ = 0;
}

void LeadlagController::Init(const controller::LeadlagConf &leadlag_conf,
                             const double dt) {
  previous_output_ = 0.0;
  previous_innerstate_ = 0.0;
  innerstate_ = 0.0;
  innerstate_saturation_high_ =
      std::fabs(leadlag_conf.innerstate_saturation_level());
  innerstate_saturation_low_ =
      -std::fabs(leadlag_conf.innerstate_saturation_level());
  innerstate_saturation_status_ = 0;
  SetLeadlag(leadlag_conf);
  TransformC2d(dt);
}

void LeadlagController::SetLeadlag(
    const controller::LeadlagConf &leadlag_conf) {
  alpha_ = leadlag_conf.alpha();
  beta_ = leadlag_conf.beta();
  tau_ = leadlag_conf.tau();
}

void LeadlagController::TransformC2d(const double dt) {
  if (dt <= 0.0) {
    LOG(INFO) << "dt <= 0, continuous-discrete transformation failed, dt: "
              << dt;
    transfromc2d_enabled_ = false;
  } else {
    double a1 = alpha_ * tau_;
    double a0 = 1.00;
    double b1 = beta_ * tau_;
    double b0 = beta_;
    Ts_ = dt;
    kn1_ = 2 * b1 + Ts_ * b0;
    kn0_ = Ts_ * b0 - 2 * b1;
    kd1_ = 2 * a1 + Ts_ * a0;
    kd0_ = Ts_ * a0 - 2 * a1;
    if (kd1_ <= 0.0) {
      LOG(INFO) << "kd1 <= 0, continuous-discrete transformation failed, kd1: "
                << kd1_;
      transfromc2d_enabled_ = false;
    } else {
      transfromc2d_enabled_ = true;
    }
  }
}

int LeadlagController::InnerstateSaturationStatus() const {
  return innerstate_saturation_status_;
}
