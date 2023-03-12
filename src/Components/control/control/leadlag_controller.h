#ifndef _LEADLAG_CONTROLLER_H_
#define _LEADLAG_CONTROLLER_H_

#include "common/proto/leadlag_conf.pb.h"

/**
* @class LeadlagController
* @brief A lead/lag controller for speed and steering
using defualt integral hold
*/
class LeadlagController {
 public:
  /**
   * @brief initialize lead/lag controller
   * @param leadlag_conf configuration for leadlag controller
   * @param dt sampling time interval
   */
  void Init(const controller::LeadlagConf &leadlag_conf, const double dt);

  /**
   * alpha, beta and tau
   * @param leadlag_conf configuration for leadlag controller
   */
  void SetLeadlag(const controller::LeadlagConf &leadlag_conf);

  /**
   * @brief transfer lead/lag controller coefficients to the discrete-time form,
   with the bilinear transform (trapezoidal integration) method
   * @param dt sampling time interval
   */
  void TransformC2d(const double dt);

  /**
   * @brief reset variables for lead/leg controller
   */
  void Reset();

  /**
   * @brief compute control value based on the error
   * @param error error value, the difference between
   * a desired value and a measured value
   * @param dt sampling time interval
   * @return control value based on Lead/Lag terms
   */
  virtual double Control(const double error, const double dt);

  /**
   * @brief get saturation status
   * @return saturation status
   */
  int InnerstateSaturationStatus() const;

 protected:
  // Control coefficients in contiouous-time domain
  double alpha_ = 0.0;
  double beta_ = 0.0;
  double tau_ = 0.0;
  double Ts_ = 0.01;  // By default, control sampling time is 0.01 sec
  // Control coefficients in discrete-time domain
  double kn1_ = 0.0;
  double kn0_ = 0.0;
  double kd1_ = 0.0;
  double kd0_ = 0.0;
  // Inner (intermedia) state in discrete-time domain at Direct Form II
  double previous_output_ = 0.0;
  double previous_innerstate_ = 0.0;
  double innerstate_ = 0.0;
  double innerstate_saturation_high_ = 0.0;
  double innerstate_saturation_low_ = 0.0;
  int innerstate_saturation_status_ = 0;
  bool transfromc2d_enabled_ = false;
};

#endif
