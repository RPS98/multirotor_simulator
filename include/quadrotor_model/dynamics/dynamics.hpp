/*!*******************************************************************************************
 *  \file       dynamics.hpp
 *  \brief      Quadrotor dynamics class definition
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

#ifndef DYNAMICS_HPP
#define DYNAMICS_HPP

#include <iostream>
#include <memory>

#include "common/actuation.hpp"
#include "common/model.hpp"
#include "common/state.hpp"
#include "common/utils.hpp"

namespace quadrotor {

class Dynamics {
public:
  Dynamics(std::shared_ptr<Model> model, std::shared_ptr<State> state);
  ~Dynamics();

  void process_euler_explicit(const actuation::MotorW &actuation, const float dt);

private:
  std::shared_ptr<Model> model_;
  std::shared_ptr<State> state_;
  std::shared_ptr<State> last_state_;

  float dt_;
  Eigen::Vector4f motor_acceleration_ = Eigen::Vector4f::Zero();

private:
  static Eigen::Vector4f get_motors_angular_velocity_derivative(
      const Eigen::Vector4f &desired_angular_velocity,
      const Eigen::Vector4f &current_angular_velocity,
      const float time_constant);

  static Eigen::Vector3f get_vehicle_control_moment(
      const Eigen::Vector4f &motor_angular_velocity,
      const Eigen::Vector4f &motor_angular_velocity_derivate,
      const Eigen::Matrix<float, 3, 4> motor_thrust_coefficient_matrix,
      const Eigen::Matrix<float, 3, 4> motor_torque_coefficient_matrix,
      const Eigen::Matrix<float, 3, 4> motor_inertia_matrix);

  static Eigen::Vector3f get_vehicle_angular_velocity_derivative(
      const Eigen::Matrix3f &vehicle_inertia_matrix,
      const Eigen::Vector3f &vehicle_angular_velocity,
      const Eigen::Vector3f &vehicle_control_moment,
      const Eigen::Vector3f &vehicle_aerodynamic_moment,
      const Eigen::Vector3f &vehicle_stochastic_force,
      Eigen::Vector3f &vehicle_total_torque);

  static Eigen::Vector3f get_vehicle_position_derivative(const Eigen::Vector3f &linear_velocity);

  static Eigen::Vector4f get_vehicle_orientation_derivative(
      const Eigen::Quaternionf &orientation,
      const Eigen::Vector3f &angular_velocity);

  static Eigen::Vector3f get_vehicle_linear_velocity_derivative(
      const float mass,
      const Eigen::Quaternionf &orientation,
      const float motor_thrust_coefficient,
      const Eigen::Vector4f &motor_angular_velocity,
      const float vehicle_drag_coefficient,
      const Eigen::Vector3f &vehicle_linear_velocity,
      const Eigen::Vector3f &gravity_force,
      const Eigen::Vector3f &stochastic_force,
      Eigen::Vector3f &vehicle_total_force);

  // void motor_dynamics(Eigen::Vector4f &desired_angular_velocity);
};  // class Dynamics

}  // namespace quadrotor

#endif  // DYNAMICS_HPP