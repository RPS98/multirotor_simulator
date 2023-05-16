/*!*******************************************************************************************
 *  \file       flight_controller.hpp
 *  \brief      Flight Controller class definition
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

#ifndef FLIGHT_CONTROLLER_HPP
#define FLIGHT_CONTROLLER_HPP

#include <iostream>
#include <memory>
#include "pid_controller/PID_3D.hpp"

#include "quadrotor_model/common/actuation.hpp"
#include "quadrotor_model/common/model.hpp"
#include "quadrotor_model/common/state.hpp"

namespace quadrotor {

class FlightController {
public:
  FlightController(std::shared_ptr<Model> model,
                   Eigen::Vector3f kp,
                   Eigen::Vector3f ki,
                   Eigen::Vector3f kd);
  ~FlightController();

public:
  Eigen::Vector4f acro_to_motor_speeds(const actuation::Acro& acro,
                                       const Eigen::Vector3f& current_angular_velocity);

  void reset_controller();

private:
  std::shared_ptr<Model> model_;

  // PID controller
  pid_controller::PIDController3D pid_controller_;

  // Allocation matrix
  // [w1²]   [  thrust_c,  thrust_c,  thrust_c, thrust_c]^-1   [Ft]
  // [w2²] = [[ motors_frame_thrust_coefficient_matrix ]]    * [tx]
  // [w3²]   [[                +                       ]]      [ty]
  // [w4²]   [[ motors_frame_torque_coefficient_matrix ]]      [tz]
  Eigen::Matrix4f allocation_matrix_     = Eigen::Matrix4f::Zero();
  Eigen::Matrix4f allocation_matrix_inv_ = Eigen::Matrix4f::Zero();

  // Intermediate variables
  Eigen::Vector4f motor_speeds_angular_velocity_ = Eigen::Vector4f::Zero();
  Eigen::Vector3f angular_acceleration_          = Eigen::Vector3f::Zero();
  Eigen::Vector3f torque_desired_                = Eigen::Vector3f::Zero();

public:
  /* Getters */

  /**
   * @brief Get the pid controller object
   *
   * @return pid_controller::PIDController3D
   */
  inline pid_controller::PIDController3D get_pid_controller() const { return pid_controller_; }

  /**
   * @brief Get the allocation matrix object
   *
   * @return Eigen::Matrix4f
   */
  inline Eigen::Matrix4f get_allocation_matrix() const { return allocation_matrix_; }

  /**
   * @brief Get the allocation matrix inv object
   *
   * @return Eigen::Matrix4f
   */
  inline Eigen::Matrix4f get_allocation_matrix_inv() const { return allocation_matrix_inv_; }

  /**
   * @brief Get the motor speeds angular velocity object
   *
   * @return Eigen::Vector4f
   */
  inline Eigen::Vector4f get_motor_speeds_angular_velocity() const {
    return motor_speeds_angular_velocity_;
  }

  /**
   * @brief Get the angular acceleration object
   *
   * @return Eigen::Vector3f
   */
  inline Eigen::Vector3f get_angular_acceleration() const { return angular_acceleration_; }

  /**
   * @brief Get the torque desired object
   *
   * @return Eigen::Vector3f
   */
  inline Eigen::Vector3f get_torque_desired() const { return torque_desired_; }

};  // class FlightController

}  // namespace quadrotor

#endif  // FLIGHT_CONTROLLER_HPP