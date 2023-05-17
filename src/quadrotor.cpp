/*!*******************************************************************************************
 *  \file       quadrotor.cpp
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

#include "quadrotor_model/quadrotor.hpp"

using namespace quadrotor;

Quadrotor::Quadrotor(quadrotor_params& params) : floor_height_(params.floor_height) {
  assert(params.floor_height <= params.initial_state.kinematics.position.z());
  assert(params.vehicle_mass > 0.0f);

  model_ = std::make_shared<Model>(
      params.motor_thrust_coefficient, params.motor_torque_coefficient, params.motor_dx,
      params.motor_dy, params.motor_min_speed, params.motor_max_speed, params.motor_time_constant,
      params.motor_rotational_inertia, params.motors_frame_type, params.vehicle_mass,
      params.vehicle_inertia, params.vehicle_drag_coefficient,
      params.vehicle_aero_moment_coefficient, params.gravity,
      params.moment_process_noise_auto_correlation, params.force_process_noise_auto_correlation);

  state_ = std::make_shared<State>(params.initial_state);

  dynamics_ = std::make_shared<Dynamics>(model_, state_);

  flight_controller_ = std::make_shared<FlightController>(model_, params.kp, params.ki, params.kd,
                                                          params.antiwindup_cte, params.alpha);

  imu_ = std::make_shared<IMU>(params.gyro_noise_var, params.accel_noise_var,
                               params.gyro_bias_noise_autocorr_time,
                               params.accel_bias_noise_autocorr_time,
                               params.initial_state.kinematics.orientation, params.imu_orientation);

  floor_force_ =
      Eigen::Vector3f(0.0f, 0.0f, -1.0f * model_->get_gravity_force().z() * model_->get_mass());
}

Quadrotor::~Quadrotor() {
  // TODO
  return;
}

void Quadrotor::update(float dt, const actuation::Acro& actuation) {
  if (dt <= 0.0f) {
    std::cerr << "Quadrotor::update: dt must be greater than 0" << std::endl;
    return;
  }

  // Compute floor force
  apply_floor_force();

  // Convert from Acro to MotorW
  acro_to_motor_speeds(actuation, dt);

  // Update dynamics
  process_euler_explicit(dt);

  // Apply floor limit
  apply_floor_limit();

  // Update IMU
  update_imu(dt);
}

inline void Quadrotor::apply_floor_force() {
  // Compute floor force
  if (state_->kinematics.position.z() <= floor_height_) {
    external_force_ = floor_force_;
  } else {
    external_force_ = Eigen::Vector3f::Zero();
  }
}

inline void Quadrotor::acro_to_motor_speeds(const actuation::Acro& actuation, const float dt) {
  // Convert from Acro to MotorW
  actuation_motor_w_.angular_velocity =
      flight_controller_->acro_to_motor_speeds(actuation, state_->kinematics.angular_velocity, dt);
}

inline void Quadrotor::process_euler_explicit(float dt) {
  // Update dynamics
  dynamics_->process_euler_explicit(actuation_motor_w_, dt, external_force_);
}

inline void Quadrotor::apply_floor_limit() {
  // Apply floor limit
  if (state_->kinematics.position.z() <= floor_height_) {
    state_->kinematics.position.z()        = floor_height_;
    state_->kinematics.linear_velocity.z() = std::max(0.0f, state_->kinematics.linear_velocity.z());
  }
}

inline void Quadrotor::update_imu(float dt) {
  // Fs = R_(w->b) * (Ft - g - Fext)
  Eigen::Vector3f specific_force =
      state_->kinematics.orientation.inverse() *
      (state_->dynamics.force - model_->get_gravity_force() - external_force_);
  imu_->update(dt, specific_force, state_->kinematics.angular_velocity);
}