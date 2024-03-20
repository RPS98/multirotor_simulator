/*!*******************************************************************************************
 *  \file       trajectory_controller.hpp
 *  \brief      TrajectoryController class definition
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

#ifndef MULTIROTOR_CONTROLLERS_CONTROLLERS_TRAJECTORY_CONTROLLER_HPP_
#define MULTIROTOR_CONTROLLERS_CONTROLLERS_TRAJECTORY_CONTROLLER_HPP_

#include "pid_controller/pid.hpp"

namespace trajectory_controller {

/**
 * @brief Trajectory controller parameters
 *
 * @tparam P Precision type of the controller
 */
template <typename P = double>
struct TrajectoryControllerParams {
  // Check if P is a numeric type
  static_assert(std::is_floating_point<P>::value,
                "MotorParams must be used with a floating-point type");

  using PIDParams = pid_controller::PIDParams<P>;

  PIDParams pid_params = PIDParams();  // PID parameters for trajectory control
};

/**
 * @brief Trajectory controller class
 *
 * Convert a desired trajectory into a desired acceleration
 *
 * @tparam P Precision type of the controller
 */
template <typename P = double>
class TrajectoryController {
  // Check if P is a numeric type
  static_assert(std::is_floating_point<P>::value,
                "MotorParams must be used with a floating-point type");

  using Scalar     = P;
  using Vector3    = Eigen::Matrix<P, 3, 1>;
  using Matrix3    = Eigen::Matrix<P, 3, 3>;
  using Quaternion = Eigen::Quaternion<P>;
  using PID        = pid_controller::PID<P>;
  using PIDParams  = pid_controller::PIDParams<P>;

public:
  /**
   * @brief Construct a new Geometric Controller object
   *
   * @param vehicle_mass Scalar with the vehicle mass
   * @param kp_rot Matrix3 with the proportional gains for the rotation
   * @param pid_params PID parameters
   * @param gravity Vector3 with the gravity vector
   */
  explicit TrajectoryController(const PIDParams& pid_params) : pid_(pid_params) {}

  /**
   * @brief Construct a new Geometric Controller object
   *
   * @param params ControllerParams
   */
  explicit TrajectoryController(
      const TrajectoryControllerParams<P>& params = TrajectoryControllerParams<P>())
      : pid_(params.pid_params) {}

  /**
   * @brief Destroy the Geometric Controller object
   *
   */
  ~TrajectoryController() {}

  /**
   * @brief Convert a desired trajectory into a desired attitude and thrust using differential
   * flatness
   *
   * @param current_position Vector3 with the current position (m) in earth frame
   * @param current_velocity Vector3 with the current velocity (m/s) in earth frame
   * @param desired_position Vector3 with the desired position (m) in earth frame
   * @param desired_velocity Vector3 with the desired velocity (m/s) in earth frame
   * @param desired_acceleration Vector3 with the desired acceleration (m/s^2) in earth frame
   * @param dt Scalar with the time step (s)
   *
   * @return Vector3 with the desired linear acceleration (m/s^2) in earth frame
   */
  Vector3 trajectory_to_linear_acceleration(const Vector3& current_position,
                                            const Vector3& current_velocity,
                                            const Vector3& desired_position,
                                            const Vector3& desired_velocity,
                                            const Vector3& desired_acceleration,
                                            const Scalar dt) {
    // Compute trajectory errors
    position_error_ = pid_.get_error(current_position, desired_position);
    velocity_error_ = pid_.get_error(current_velocity, desired_velocity);

    // Compute desired acceleration
    desired_acceleration_ =
        pid_.compute_control(dt, position_error_, velocity_error_) + desired_acceleration;

    return desired_acceleration_;
  }

  /**
   * @brief Update PID parameters
   *
   * @param pid_params PID parameters
   */
  inline void update_pid_params(const PIDParams& pid_params) { pid_.update_params(pid_params); }

  /**
   * @brief Update controller parameters
   *
   * @param params TrajectoryControllerParams
   */
  inline void update_params(const TrajectoryControllerParams<P>& params) {
    update_pid_params(params.pid_params);
  }

  /**
   * @brief Reset controller
   */
  inline void reset_controller() { pid_.reset_controller(); }

  // Getters

  /**
   * @brief Get the PID
   *
   * @return PID
   */
  inline PID get_pid() const { return pid_; }

  /**
   * @brief Get the PID
   *
   * @return const PID&
   */
  inline const PID& get_pid_const() const { return pid_; }

  /**
   * @brief Get the desired linear acceleration
   *
   * @return Vector3 Desired linear acceleration (m/s^2)
   */
  inline Vector3 get_desired_linear_acceleration() const { return desired_acceleration_; }

  /**
   * @brief Get the desired linear acceleration
   *
   * @return const Vector3& Desired linear acceleration (m/s^2)
   */
  inline const Vector3& get_desired_linear_acceleration_const() const {
    return desired_acceleration_;
  }

  /**
   * @brief Get the position error
   *
   * @return Vector3 Position error (m)
   */
  inline Vector3 get_position_error() const { return position_error_; }

  /**
   * @brief Get the position error
   *
   * @return const Vector3& Position error (m)
   */
  inline const Vector3& get_position_error_const() const { return position_error_; }

  /**
   * @brief Get the velocity error
   *
   * @return Vector3 Velocity error (m/s)
   */
  inline Vector3 get_velocity_error() const { return velocity_error_; }

  /**
   * @brief Get the velocity error
   *
   * @return const Vector3& Velocity error (m/s)
   */
  inline const Vector3& get_velocity_error_const() const { return velocity_error_; }

protected:
  // PID controller
  PID pid_;

  // Internal variables
  Vector3 position_error_       = Vector3::Zero();  // Position error (m)
  Vector3 velocity_error_       = Vector3::Zero();  // Velocity error (m/s)
  Vector3 desired_acceleration_ = Vector3::Zero();  // Desired linear acceleration (m/s^2)
};
}  // namespace trajectory_controller

#endif  // MULTIROTOR_CONTROLLERS_CONTROLLERS_TRAJECTORY_CONTROLLER_HPP_
