/*!*******************************************************************************************
 *  \file       position_controller.hpp
 *  \brief      PositionController class definition.
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

#ifndef MULTIROTOR_CONTROLLERS_CONTROLLERS_POSITION_CONTROLLER_HPP_
#define MULTIROTOR_CONTROLLERS_CONTROLLERS_POSITION_CONTROLLER_HPP_

#include "pid_controller/pid.hpp"

namespace position_controller {

/**
 * @brief Position controller parameters
 *
 * @tparam P Precision type of the controller
 */
template <typename P = double>
struct PositionControllerParams {
  // Check if P is a numeric type
  static_assert(std::is_floating_point<P>::value,
                "MotorParams must be used with a floating-point type");

  using PIDParams = pid_controller::PIDParams<P>;

  PIDParams pid_params = PIDParams();  // PID parameters
};

/**
 * @brief Position controller using PID controller
 *
 * Convert the desired position to desired linear velocity.
 *
 * @tparam P Precision type of the controller
 */
template <typename P = double>
class PositionController : public pid_controller::PID<P> {
  // Check if P is a numeric type
  static_assert(std::is_floating_point<P>::value,
                "PositionController must be used with a floating-point type");

  using Scalar    = P;
  using Vector3   = Eigen::Matrix<P, 3, 1>;
  using Matrix3   = Eigen::Matrix<P, 3, 3>;
  using PID       = pid_controller::PID<P>;
  using PIDParams = pid_controller::PIDParams<P>;

public:
  /**
   * @brief Construct a new PositionController object
   *
   * @param pid_params PID parameters
   */
  explicit PositionController(const PIDParams &pid_params) : PID(pid_params) {}

  /**
   * @brief Construct a new PositionController object
   *
   * @param params PositionControllerParams parameters
   */
  explicit PositionController(
      const PositionControllerParams<P> &params = PositionControllerParams<P>())
      : PID(params.pid_params) {}

  /**
   * @brief Destroy the Position Controller object
   *
   */
  ~PositionController() {}

  /**
   * @brief Compute the control action
   *
   * @param current_position Current position (m)
   * @param desired_position Desired position (m)
   * @param dt Scalar with the time step (s)
   *
   * @return Vector3 with the desired vehicle linear velocity (m/s)
   */
  Vector3 position_to_linear_velocity(const Vector3 &current_position,
                                      const Vector3 &desired_position,
                                      const Scalar dt) {
    // Compute the position error
    Vector3 position_error = this->get_error(current_position, desired_position);

    // Compute the desired linear velocity
    Vector3 desired_linear_velocity = this->compute_control(dt, position_error);

    return desired_linear_velocity;
  }

  /**
   * @brief Update PID parameters
   *
   * @param pid_params PID parameters
   */
  inline void update_pid_params(const PIDParams &params) { PID::update_params(params); }

  /**
   * @brief Update controller parameters
   *
   * @param params PositionControllerParams
   */
  void update_params(const PositionControllerParams<P> &params) {
    PID::update_params(params.pid_params);
  }

  // Getters

  /**
   * @brief Get the desired linear velocity
   *
   * @return Vector3 Desired linear velocity (m/s)
   */
  inline Vector3 get_desired_linear_velocity() const { return PID::get_output(); }

  /**
   * @brief Get the desired position error
   *
   * @return Vector3 Desired position error (m)
   */
  inline Vector3 get_position_error() const { return PID::get_proportional_error(); }
};  // Class PositionController

}  // namespace position_controller

#endif  // MULTIROTOR_CONTROLLERS_CONTROLLERS_POSITION_CONTROLLER_HPP_
