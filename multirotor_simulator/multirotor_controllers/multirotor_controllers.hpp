/*!*******************************************************************************************
 *  \file       multirotor_controller.hpp
 *  \brief      Controller class definition
 *  \authors    Rafael Pérez Seguí
 *
 *  \copyright  Copyright (c) 2022 Universidad Politécnica de Madrid
 *              All Rights Reserved
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ********************************************************************************/

#ifndef MULTIROTOR_CONTROLLERS_MULTIROTOR_CONTROLLERS_HPP_
#define MULTIROTOR_CONTROLLERS_MULTIROTOR_CONTROLLERS_HPP_

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <utility>

#include "multirotor_controllers/controllers/acro_controller.hpp"
#include "multirotor_controllers/controllers/indi_controller.hpp"
#include "multirotor_controllers/controllers/position_controller.hpp"
#include "multirotor_controllers/controllers/trajectory_controller.hpp"
#include "multirotor_controllers/controllers/velocity_controller.hpp"

namespace multirotor {

namespace controller {

/**
 * @brief Controller parameters
 *
 * @tparam P Precision
 * @tparam num_rotors Number of rotors
 */
template <typename P = double, int num_rotors = 4>
struct ControllerParams {
  static_assert(std::is_floating_point<P>::value,
                "MotorParams must be used with a floating-point type");
  static_assert(num_rotors > 0, "MotorParams must be used with a positive number of rotors");

  using AcroControllerParams       = acro_controller::AcroControllerParams<P>;
  using IndiControllerParameters   = indi_controller::IndiControllerParams<P, num_rotors>;
  using PositionControllerParams   = position_controller::PositionControllerParams<P>;
  using TrajectoryControllerParams = trajectory_controller::TrajectoryControllerParams<P>;
  using VelocityControllerParams   = velocity_controller::VelocityControllerParams<P>;

  AcroControllerParams acro_controller_params =
      AcroControllerParams();  // Attitude controller parameters
  IndiControllerParameters indi_controller_params =
      IndiControllerParameters();  // Indi controller parameters
  PositionControllerParams position_controller_params =
      PositionControllerParams();  // Position controller parameters
  TrajectoryControllerParams trajectory_controller_params =
      TrajectoryControllerParams();  // Trajectory controller parameters
  VelocityControllerParams velocity_controller_params =
      VelocityControllerParams();  // Velocity controller parameters
};

/**
 * @brief Controller class
 *
 * @tparam P Precision
 * @tparam num_rotors Number of rotors
 */
template <typename P = double, int num_rotors = 4>
class Controller {
  using AcroController             = acro_controller::AcroController<P>;
  using AcroControllerParams       = acro_controller::AcroControllerParams<P>;
  using IndiController             = indi_controller::IndiController<P, num_rotors>;
  using IndiControllerParameters   = indi_controller::IndiControllerParams<P, num_rotors>;
  using PositionController         = position_controller::PositionController<P>;
  using PositionControllerParams   = position_controller::PositionControllerParams<P>;
  using TrajectoryController       = trajectory_controller::TrajectoryController<P>;
  using TrajectoryControllerParams = trajectory_controller::TrajectoryControllerParams<P>;
  using VelocityController         = velocity_controller::VelocityController<P>;
  using VelocityControllerParams   = velocity_controller::VelocityControllerParams<P>;

  using Scalar     = P;
  using Vector3    = Eigen::Matrix<P, 3, 1>;
  using Vector6    = Eigen::Matrix<P, 6, 1>;
  using VectorN    = Eigen::Matrix<P, num_rotors, 1>;
  using Matrix3    = Eigen::Matrix<P, 3, 3>;
  using MatrixN    = Eigen::Matrix<P, num_rotors, 6>;
  using Quaternion = Eigen::Quaternion<P>;
  using PID        = pid_controller::PID<P>;
  using PIDParams  = pid_controller::PIDParams<P>;

public:
  /**
   * @brief Construct a new Controller object
   *
   * @param indi_controller_params INDI controller parameters
   */
  Controller(
      const AcroControllerParams &acro_controller_params             = AcroControllerParams(),
      const IndiControllerParameters &indi_controller_params         = IndiControllerParameters(),
      const PositionControllerParams &position_controller_params     = PositionControllerParams(),
      const TrajectoryControllerParams &trajectory_controller_params = TrajectoryControllerParams(),
      const VelocityControllerParams &velocity_controller_params     = VelocityControllerParams())
      : acro_controller_(acro_controller_params), indi_controller_(indi_controller_params),
        position_controller_(position_controller_params),
        trajectory_controller_(trajectory_controller_params),
        velocity_controller_(velocity_controller_params) {}

  /**
   * @brief Construct a new Controller object
   *
   * @param controller_params Controller parameters
   */
  explicit Controller(const ControllerParams<P, num_rotors> &controller_params)
      : Controller(controller_params.acro_controller_params,
                   controller_params.indi_controller_params,
                   controller_params.position_controller_params,
                   controller_params.trajectory_controller_params,
                   controller_params.velocity_controller_params) {}

  ~Controller() {}

  /**
   * @brief Compute control from Acro command to motor angular velocity
   *
   * @param current_vehicle_angular_velocity Current vehicle angular velocity (rad/s) in body frame
   * @param thrust Thrust (N) in body frame
   * @param desired_angular_velocity Desired vehicle angular velocity (rad/s) in body frame
   * @param dt Time step (s)
   *
   * @return VectorN Motor angular velocity (rad/s)
   */
  inline VectorN convert_acro_to_motor_angular_velocity(
      const Vector3 &current_vehicle_angular_velocity,
      const Scalar thrust,
      const Vector3 &desired_angular_velocity,
      const Scalar dt) {
    VectorN motor_angular_velocity = indi_controller_.acro_to_motor_angular_velocity(
        current_vehicle_angular_velocity, thrust, desired_angular_velocity, dt);
    return motor_angular_velocity;
  }

  /**
   * @brief Compute control from Linear Acceleration command to Acro command
   *
   * @param desired_acceleration Desired linear acceleration (m/s^2) in world frame
   * @param current_orientation Current orientation (Matrix3) in world frame
   * @param desired_yaw Desired yaw (rad) in world frame
   * @param dt Time step (s)
   *
   * @return std::pair<Scalar, Vector3> Output pair with the desired thrust (N) in z-axis of earth
   * frame and the desired angular velocity (rad/s) in body frame
   */
  inline std::pair<Scalar, Vector3> convert_linear_acceleration_to_acro(
      const Vector3 &desired_acceleration,
      const Matrix3 &current_orientation,
      const Scalar desired_yaw,
      const Scalar dt) {
    // Thrust in world frame
    Vector3 thrust_w = acro_controller_.acceleration_to_thrust(desired_acceleration);

    // Desired angular velocity
    Vector3 desired_vehicle_angular_velocity =
        acro_controller_.yaw_angle_to_angular_velocity(current_orientation, desired_yaw, thrust_w);

    // Thrust in body frame
    Scalar thrust_b = thrust_w.dot(current_orientation.col(2).normalized());

    if (thrust_b < 0.0) {
      thrust_b                         = 0.0;
      desired_vehicle_angular_velocity = Vector3::Zero();
    }

    return std::make_pair(thrust_b, desired_vehicle_angular_velocity);
  }

  /**
   * @brief Compute control from Trajectory command to Linear Acceleration command
   *
   * @param current_position Current position (m) in world frame
   * @param current_velocity Current velocity (m/s) in world frame
   * @param desired_position Desired position (m) in world frame
   * @param desired_velocity Desired velocity (m/s) in world frame
   * @param desired_acceleration Desired acceleration (m/s^2) in world frame
   * @param dt Time step (s)
   *
   * @return Vector3 Desired linear acceleration (m/s^2) in world frame
   */
  inline Vector3 convert_trajectory_to_linear_acceleration(const Vector3 &current_position,
                                                           const Vector3 &current_velocity,
                                                           const Vector3 &desired_position,
                                                           const Vector3 &desired_velocity,
                                                           const Vector3 &desired_acceleration,
                                                           const Scalar dt) {
    Vector3 lineal_acceleration = trajectory_controller_.trajectory_to_linear_acceleration(
        current_position, current_velocity, desired_position, desired_velocity,
        desired_acceleration, dt);
    return lineal_acceleration;
  }

  /**
   * @brief Compute control from Linear Velocity command to Linear Acceleration command
   *
   * @param current_linear_velocity Current linear velocity (m/s) in world frame
   * @param desired_linear_velocity Desired linear velocity (m/s) in world frame
   * @param dt Time step (s)
   *
   * @return Vector3 Desired linear acceleration (m/s^2) in world frame
   */
  inline Vector3 convert_linear_velocity_to_linear_acceleration(
      const Vector3 &current_linear_velocity,
      const Vector3 &desired_linear_velocity,
      const Scalar dt) {
    Vector3 linear_acceleration = velocity_controller_.linear_velocity_to_linear_acceleration(
        current_linear_velocity, desired_linear_velocity, dt);
    return linear_acceleration;
  }

  /**
   * @brief Compute control from Position command to Linear Velocity command
   *
   * @param current_position Current position (m) in world frame
   * @param desired_position Desired position (m) in world frame
   * @param dt Time step (s)
   *
   * @return Vector3 Desired linear velocity (m/s) in world frame
   */
  inline Vector3 convert_position_to_linear_velocity(const Vector3 &current_position,
                                                     const Vector3 &desired_position,
                                                     const Scalar dt) {
    Vector3 linear_velocity =
        position_controller_.position_to_linear_velocity(current_position, desired_position, dt);
    return linear_velocity;
  }

  /**
   * @brief Compute control from Acro command to motor angular velocity
   *
   * @param current_vehicle_angular_velocity Current vehicle angular velocity (rad/s) in body frame
   * @param desired_thrust Desired thrust (N) in body frame
   * @param desired_angular_velocity Desired vehicle angular velocity (rad/s) in body frame
   * @param dt Time step (s)
   *
   * @return VectorN Motor angular velocity (rad/s)
   */
  inline VectorN compute_acro_control(const Vector3 &current_vehicle_angular_velocity,
                                      const Scalar desired_thrust,
                                      const Vector3 &desired_angular_velocity,
                                      const Scalar dt) {
    desired_vehicle_angular_velocity_ = desired_angular_velocity;
    desired_thrust_                   = desired_thrust;
    desired_motor_angular_velocity_   = convert_acro_to_motor_angular_velocity(
          current_vehicle_angular_velocity, desired_thrust_, desired_vehicle_angular_velocity_, dt);
    return desired_motor_angular_velocity_;
  }

  inline VectorN compute_acceleration_control(const Matrix3 &current_orientation,
                                              const Vector3 &current_vehicle_angular_velocity,
                                              const Vector3 &desired_linear_acceleration,
                                              const Scalar desired_yaw,
                                              const Scalar dt) {
    desired_linear_acceleration_ = desired_linear_acceleration;
    std::tie(desired_thrust_, desired_vehicle_angular_velocity_) =
        convert_linear_acceleration_to_acro(desired_linear_acceleration_, current_orientation,
                                            desired_yaw, dt);
    desired_motor_angular_velocity_ = compute_acro_control(
        current_vehicle_angular_velocity, desired_thrust_, desired_vehicle_angular_velocity_, dt);
    return desired_motor_angular_velocity_;
  }

  /**
   * @brief Compute control from Trajectory command to motor angular velocity
   *
   * @param current_position Current position (m) in world frame
   * @param current_velocity Current velocity (m/s) in world frame
   * @param current_orientation Current orientation (Matrix3) in world frame
   * @param current_angular_velocity Current angular velocity (rad/s) in body frame
   * @param desired_position Desired position (m) in world frame
   * @param desired_velocity Desired velocity (m/s) in world frame
   * @param desired_acceleration Desired acceleration (m/s^2) in world frame
   * @param desired_yaw Desired yaw (rad) in world frame
   * @param dt Time step (s)
   *
   * @return VectorN Motor angular velocity (rad/s)
   */
  inline VectorN compute_trajectory_control(const Vector3 &current_position,
                                            const Vector3 &current_velocity,
                                            const Matrix3 &current_orientation,
                                            const Vector3 &current_angular_velocity,
                                            const Vector3 &desired_position,
                                            const Vector3 &desired_velocity,
                                            const Vector3 &desired_acceleration,
                                            const Scalar desired_yaw,
                                            const Scalar dt) {
    desired_position_            = desired_position;
    desired_linear_velocity_     = desired_velocity;
    desired_linear_acceleration_ = convert_trajectory_to_linear_acceleration(
        current_position, current_velocity, desired_position, desired_velocity,
        desired_acceleration, dt);
    desired_motor_angular_velocity_ =
        compute_acceleration_control(current_orientation, current_angular_velocity,
                                     desired_linear_acceleration_, desired_yaw, dt);
    return desired_motor_angular_velocity_;
  }

  /**
   * @brief Compute control from Velocity command to motor angular velocity
   *
   * @param current_linear_velocity Current linear velocity (m/s) in world frame
   * @param current_orientation Current orientation (Matrix3) in world frame
   * @param current_angular_velocity Current angular velocity (rad/s) in body frame
   * @param desired_linear_velocity Desired linear velocity (m/s) in world frame
   * @param desired_yaw Desired yaw (rad) in world frame
   * @param dt Time step (s)
   *
   * @return VectorN Motor angular velocity (rad/s)
   */
  inline VectorN compute_velocity_control(const Vector3 &current_linear_velocity,
                                          const Matrix3 &current_orientation,
                                          const Vector3 &current_angular_velocity,
                                          const Vector3 &desired_linear_velocity,
                                          const Scalar desired_yaw,
                                          const Scalar dt) {
    desired_linear_velocity_     = desired_linear_velocity;
    desired_linear_acceleration_ = convert_linear_velocity_to_linear_acceleration(
        current_linear_velocity, desired_linear_velocity, dt);
    desired_motor_angular_velocity_ =
        compute_acceleration_control(current_orientation, current_angular_velocity,
                                     desired_linear_acceleration_, desired_yaw, dt);
    return desired_motor_angular_velocity_;
  }

  /**
   * @brief Compute control from Position command to motor angular velocity
   *
   * @param current_position Current position (m) in world frame
   * @param current_orientation Current orientation (Matrix3) in world frame
   * @param current_angular_velocity Current angular velocity (rad/s) in body frame
   * @param desired_position Desired position (m) in world frame
   * @param desired_yaw Desired yaw (rad) in world frame
   * @param dt Time step (s)
   *
   * @return VectorN Motor angular velocity (rad/s)
   */
  inline VectorN compute_position_control(const Vector3 &current_position,
                                          const Matrix3 &current_orientation,
                                          const Vector3 &current_linear_velocity,
                                          const Vector3 &current_angular_velocity,
                                          const Vector3 &desired_position,
                                          const Scalar desired_yaw,
                                          const Scalar dt) {
    desired_position_ = desired_position;
    desired_linear_velocity_ =
        convert_position_to_linear_velocity(current_position, desired_position, dt);
    desired_motor_angular_velocity_ = compute_velocity_control(
        current_linear_velocity, current_orientation, current_angular_velocity,
        desired_linear_velocity_, desired_yaw, dt);
    return desired_motor_angular_velocity_;
  }

  /**
   * @brief Reset controller
   */
  inline void reset_controller() {
    indi_controller_.reset_controller();
    position_controller_.reset_controller();
    trajectory_controller_.reset_controller();
    velocity_controller_.reset_controller();
  }

  /**
   * @brief Update attitude controller parameters
   *
   * @param acro_controller_params
   */
  inline void update_acro_controller_params(const AcroControllerParams &acro_controller_params) {
    acro_controller_.update_params(acro_controller_params);
  }

  /**
   * @brief Update indi controller parameters
   *
   * @param indi_controller_params
   */
  inline void update_indi_controller_params(
      const IndiControllerParameters &indi_controller_params) {
    indi_controller_.update_params(indi_controller_params);
  }

  /**
   * @brief Update position controller parameters
   *
   * @param position_controller_params
   */
  inline void update_position_controller_params(
      const PositionControllerParams &position_controller_params) {
    position_controller_.update_params(position_controller_params);
  }

  /**
   * @brief Update trajectory controller parameters
   *
   * @param trajectory_controller_params
   */
  inline void update_trajectory_controller_params(
      const TrajectoryControllerParams &trajectory_controller_params) {
    trajectory_controller_.update_params(trajectory_controller_params);
  }

  /**
   * @brief Update velocity controller parameters
   *
   * @param velocity_controller_params
   */
  inline void update_velocity_controller_params(
      const VelocityControllerParams &velocity_controller_params) {
    velocity_controller_.update_params(velocity_controller_params);
  }

  /**
   * @brief Update controller parameters
   *
   * @param controller_params
   */
  inline void update_params(const ControllerParams<P, num_rotors> &controller_params) {
    update_acro_controller_params(controller_params.acro_controller_params);
    update_indi_controller_params(controller_params.indi_controller_params);
    update_position_controller_params(controller_params.position_controller_params);
    update_trajectory_controller_params(controller_params.trajectory_controller_params);
    update_velocity_controller_params(controller_params.velocity_controller_params);
  }

  // Getters

  /**
   * @brief Get attitude controller
   *
   * @return AcroController
   */
  inline AcroController get_acro_controller() const { return acro_controller_; }

  /**
   * @brief Get attitude controller
   *
   * @return const AcroController&
   */
  inline const AcroController &get_acro_controller_const() const { return acro_controller_; }

  /**
   * @brief Get indi controller
   *
   * @return IndiController
   */
  inline IndiController get_indi_controller() const { return indi_controller_; }

  /**
   * @brief Get indi controller
   *
   * @return const IndiController&
   */
  inline const IndiController &get_indi_controller_const() const { return indi_controller_; }

  /**
   * @brief Get position controller
   *
   * @return PositionController
   */
  inline PositionController get_position_controller() const { return position_controller_; }

  /**
   * @brief Get position controller
   *
   * @return const PositionController&
   */
  inline const PositionController &get_position_controller_const() const {
    return position_controller_;
  }

  /**
   * @brief Get trajectory controller
   *
   * @return TrajectoryController
   */
  inline TrajectoryController get_trajectory_controller() const { return trajectory_controller_; }

  /**
   * @brief Get trajectory controller
   *
   * @return const TrajectoryController&
   */
  inline const TrajectoryController &get_trajectory_controller_const() const {
    return trajectory_controller_;
  }

  /**
   * @brief Get velocity controller
   *
   * @return VelocityController
   */
  inline VelocityController get_velocity_controller() const { return velocity_controller_; }

  /**
   * @brief Get velocity controller
   *
   * @return const VelocityController&
   */
  inline const VelocityController &get_velocity_controller_const() const {
    return velocity_controller_;
  }

  /**
   * @brief Get desired motor angular velocity
   *
   * @return VectorN
   */
  inline VectorN get_desired_motor_angular_velocity() const {
    return desired_motor_angular_velocity_;
  }

  /**
   * @brief Get desired motor angular velocity
   *
   * @return const VectorN&
   */
  inline const VectorN &get_desired_motor_angular_velocity_const() const {
    return desired_motor_angular_velocity_;
  }

  /**
   * @brief Get desired thrust
   *
   * @return Scalar
   */
  inline Scalar get_desired_thrust() const { return desired_thrust_; }

  /**
   * @brief Get desired vehicle angular velocity
   *
   * @return Vector3
   */
  inline Vector3 get_desired_vehicle_angular_velocity() const {
    return desired_vehicle_angular_velocity_;
  }

  /**
   * @brief Get desired vehicle angular velocity const
   *
   * @return const Vector3&
   */
  inline const Vector3 &get_desired_vehicle_angular_velocity_const() const {
    return desired_vehicle_angular_velocity_;
  }

  /**
   * @brief Get desired linear acceleration
   *
   * @return Vector3
   */
  inline Vector3 get_desired_linear_acceleration() const { return desired_linear_acceleration_; }

  /**
   * @brief Get desired linear acceleration
   *
   * @return const Vector3&
   */
  inline const Vector3 &get_desired_linear_acceleration_const() const {
    return desired_linear_acceleration_;
  }

  /**
   * @brief Get desired linear velocity
   *
   * @return Vector3
   */
  inline Vector3 get_desired_linear_velocity() const { return desired_linear_velocity_; }

  /**
   * @brief Get desired linear velocity
   *
   * @return const Vector3&
   */
  inline const Vector3 &get_desired_linear_velocity_const() const {
    return desired_linear_velocity_;
  }

  /**
   * @brief Get desired position
   *
   * @return Vector3
   */
  inline Vector3 get_desired_position() const { return desired_position_; }

  /**
   * @brief Get desired position
   *
   * @return const Vector3&
   */
  inline const Vector3 &get_desired_position_const() const { return desired_position_; }

private:
  AcroController acro_controller_;
  IndiController indi_controller_;
  PositionController position_controller_;
  TrajectoryController trajectory_controller_;
  VelocityController velocity_controller_;

  Vector3 desired_position_                 = Vector3::Zero();  // Desired position (m)
  Vector3 desired_linear_velocity_          = Vector3::Zero();  // Desired velocity (m/s)
  Vector3 desired_linear_acceleration_      = Vector3::Zero();  // Desired acceleration (m/s^2)
  Scalar desired_thrust_                    = 0.0;              // Thrust (N)
  Vector3 desired_vehicle_angular_velocity_ = Vector3::Zero();  // Angular velocity (rad/s)
  VectorN desired_motor_angular_velocity_   = VectorN::Zero();  // Motor angular velocity (rad/s)
};

}  // namespace controller

}  // namespace multirotor

#endif  // MULTIROTOR_CONTROLLERS_MULTIROTOR_CONTROLLERS_HPP_
