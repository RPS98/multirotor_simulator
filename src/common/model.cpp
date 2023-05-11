/*!*******************************************************************************************
 *  \file       quadrotor_model.cpp
 *  \brief      Quadrotor model implementation file.
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

#include "quadrotor_model/common/model.hpp"

using namespace quadrotor;

Model::Model(float motor_thrust_coefficient,
             float motor_torque_coefficient,
             float motor_dx,
             float motor_dy,
             float motor_min_speed,
             float motor_max_speed,
             float motor_time_constant,
             float motor_rotational_inertia,
             float vehicle_mass,
             const Eigen::Matrix3f& vehicle_inertia,
             float vehicle_drag_coefficient,
             Eigen::Matrix3f vehicle_aero_moment_coefficient,
             const Eigen::Vector3f& gravity,
             float moment_process_noise_auto_correlation,
             float force_process_noise_auto_correlation)
    : motor_thrust_coefficient(motor_thrust_coefficient),
      motor_torque_coefficient(motor_torque_coefficient), motor_min_speed(motor_min_speed),
      motor_max_speed(motor_max_speed), motor_time_constant(motor_time_constant),
      motor_rotational_inertia(motor_rotational_inertia), vehicle_mass(vehicle_mass),
      vehicle_inertia(vehicle_inertia), vehicle_drag_coefficient(vehicle_drag_coefficient),
      vehicle_aero_moment_coefficient(vehicle_aero_moment_coefficient), gravity(gravity),
      moment_process_noise_auto_correlation(moment_process_noise_auto_correlation),
      force_process_noise_auto_correlation(force_process_noise_auto_correlation) {
  // Frame transformations
  motors_frame_thrust_coefficient_matrix << 0.0f, motor_dy * motor_thrust_coefficient, 0.0f,
      -motor_dy * motor_thrust_coefficient, -motor_dx * motor_thrust_coefficient, 0.0f,
      motor_dx * motor_thrust_coefficient, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f;

  motors_frame_torque_coefficient_matrix << 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
      motor_torque_coefficient, -motor_torque_coefficient, motor_torque_coefficient,
      -motor_torque_coefficient;

  motors_frame_inertia_matrix << 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
      motor_rotational_inertia, -motor_rotational_inertia, motor_rotational_inertia,
      -motor_rotational_inertia;

  // Random number generator
  random_number_generator.seed(std::chrono::system_clock::now().time_since_epoch().count());
}

Model::~Model() {
  // TODO
}

void Model::set_stochastic_noise(const float dt) {
  stoch_force  = Eigen::Vector3f(sqrt(force_process_noise_auto_correlation / dt) *
                                     standard_normal_distribution(random_number_generator),
                                 sqrt(force_process_noise_auto_correlation / dt) *
                                     standard_normal_distribution(random_number_generator),
                                 sqrt(force_process_noise_auto_correlation / dt) *
                                     standard_normal_distribution(random_number_generator));
  stoch_moment = Eigen::Vector3f(sqrt(moment_process_noise_auto_correlation / dt) *
                                     standard_normal_distribution(random_number_generator),
                                 sqrt(moment_process_noise_auto_correlation / dt) *
                                     standard_normal_distribution(random_number_generator),
                                 sqrt(moment_process_noise_auto_correlation / dt) *
                                     standard_normal_distribution(random_number_generator));
}

Eigen::Vector3f Model::get_motor_torque(const Eigen::Vector4f& motors_angular_velocity,
                                        const Eigen::Vector4f& motors_angular_acceleration) const {
  Eigen::Vector4f motors_angular_velocity_squared = motors_angular_velocity.array().square();

  Eigen::Vector3f motor_torque_angular_velocity =
      (motors_frame_thrust_coefficient_matrix + motors_frame_torque_coefficient_matrix) *
      motors_angular_velocity_squared;

  Eigen::Vector3f motor_torque_angular_acceleration =
      motors_frame_inertia_matrix * motors_angular_acceleration;

  Eigen::Vector3f motor_torque = motor_torque_angular_velocity + motor_torque_angular_acceleration;

  return motor_torque;
}