#ifndef _VEHICLE_STATE_PROVIDER_H_
#define _VEHICLE_STATE_PROVIDER_H_

#include <memory>
#include <string>

#include "common/config/flags.h"
#include "common/math/box2d.h"
#include "common/math/vec2d.h"
#include "common/proto/chassis.pb.h"
#include "common/proto/localization.pb.h"
#include "common/proto/vehicle_state.pb.h"
#include "status.h"

/**
 * @class VehicleStateProvider
 * @brief The class of vehicle state.
 *        It includes basic information and computation
 *        about the state of the vehicle.
 */
class VehicleStateProvider {
 public:
  /**
   * @brief Constructor by information of localization and chassis.
   * @param localization Localization information of the vehicle.
   * @param chassis Chassis information of the vehicle.
   */
  Status Update(const controller::LocalizationEstimate& localization,
                const canbus::Chassis& chassis);

  /**
   * @brief Update VehicleStateProvider instance by protobuf files.
   * @param localization_file the localization protobuf file.
   * @param chassis_file The chassis protobuf file
   */
  void Update(const std::string& localization_file,
              const std::string& chassis_file);

  double timestamp() const;

  const controller::Pose& pose() const;
  const controller::Pose& original_pose() const;

  /**
   * @brief Default destructor.
   */
  virtual ~VehicleStateProvider() = default;

  /**
   * @brief Get the x-coordinate of vehicle position.
   * @return The x-coordinate of vehicle position.
   */
  double x() const;

  /**
   * @brief Get the y-coordinate of vehicle position.
   * @return The y-coordinate of vehicle position.
   */
  double y() const;

  /**
   * @brief Get the z coordinate of vehicle position.
   * @return The z coordinate of vehicle position.
   */
  double z() const;

  /**
   * @brief Get the kappa of vehicle position.
   *  the positive or negative sign is decided by the vehicle heading vector
   *  along the path
   * @return The kappa of vehicle position.
   */
  double kappa() const;

  /**
   * @brief Get the vehicle roll angle.
   * @return The euler roll angle.
   */
  double roll() const;

  /**
   * @brief Get the vehicle pitch angle.
   * @return The euler pitch angle.
   */
  double pitch() const;

  /**
   * @brief Get the vehicle yaw angle.
   *  As of now, use the heading instead of yaw angle.
   *  Heading angle with East as zero, yaw angle has North as zero
   * @return The euler yaw angle.
   */
  double yaw() const;

  /**
   * @brief Get the heading of vehicle position, which is the angle
   *        between the vehicle's heading direction and the x-axis.
   * @return The angle between the vehicle's heading direction
   *         and the x-axis.
   */
  double heading() const;

  /**
   * @brief Get the vehicle's linear velocity.
   * @return The vehicle's linear velocity.
   */
  double linear_velocity() const;

  /**
   * @brief Get the vehicle's angular velocity.
   * @return The vehicle's angular velocity.
   */
  double angular_velocity() const;

  /**
   * @brief Get the vehicle's linear acceleration.
   * @return The vehicle's linear acceleration.
   */
  double linear_acceleration() const;

  /**
   * @brief Get the vehicle's gear position.
   * @return The vehicle's gear position.
   */
  double gear() const;

  /**
   * @brief Get the vehicle's steering angle.
   * @return double
   */
  double steering_percentage() const;

  /**
   * @brief Set the vehicle's linear velocity.
   * @param linear_velocity The value to set the vehicle's linear velocity.
   */
  void set_linear_velocity(const double linear_velocity);

  /**
   * @brief Estimate future position from current position and heading,
   *        along a period of time, by constant linear velocity,
   *        linear acceleration, angular velocity.
   * @param t The length of time period.
   * @return The estimated future position in time t.
   */
  Vec2d EstimateFuturePosition(const double t) const;

  /**
   * @brief Compute the position of center of mass(COM) of the vehicle,
   *        given the distance from rear wheels to the center of mass.
   * @param rear_to_com_distance Distance from rear wheels to
   *        the vehicle's center of mass.
   * @return The position of the vehicle's center of mass.
   */
  Vec2d ComputeCOMPosition(const double rear_to_com_distance) const;

  const vehicle_state::VehicleState& vehicle_state() const;

 private:
  bool ConstructExceptLinearVelocity(
      const controller::LocalizationEstimate& localization);

  vehicle_state::VehicleState vehicle_state_;
  controller::LocalizationEstimate original_localization_;
};

#endif
