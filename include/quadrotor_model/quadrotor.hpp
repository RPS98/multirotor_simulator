/*!*******************************************************************************************
 *  \file       quadrotor.hpp
 *  \brief      Quadrotor class definition
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

#ifndef QUADROTOR_HPP
#define QUADROTOR_HPP

#include <iostream>
#include <memory>

#include "quadrotor_model/common/actuation.hpp"
#include "quadrotor_model/common/model.hpp"
#include "quadrotor_model/common/state.hpp"
#include "quadrotor_model/dynamics/dynamics.hpp"
#include "quadrotor_model/flight_controller/flight_controller.hpp"
#include "quadrotor_model/imu/imu.hpp"

namespace quadrotor {

struct quadrotor_params {
  /* Quadrotor Parameters */
  float floor_height = 0.0f;

  /* Model Parameters */
  float motor_thrust_coefficient;
  float motor_torque_coefficient;
  float motor_dx;
  float motor_dy;
  float motor_min_speed;
  float motor_max_speed;
  float motor_time_constant;
  float motor_rotational_inertia;
  float vehicle_mass;
  Eigen::Matrix3f vehicle_inertia;
  float vehicle_drag_coefficient                  = 0.0f;
  Eigen::Matrix3f vehicle_aero_moment_coefficient = Eigen::Matrix3f::Zero();
  Eigen::Vector3f gravity                         = Eigen::Vector3f(0.0f, 0.0f, -9.81f);
  float moment_process_noise_auto_correlation     = 0.0f;
  float force_process_noise_auto_correlation      = 0.0f;

  /* State */
  State initial_state;

  /* Flight Controller Parameters */
  Eigen::Vector3f kp;
  Eigen::Vector3f ki = Eigen::Vector3f::Zero();
  Eigen::Vector3f kd = Eigen::Vector3f::Zero();

  /* IMU Parameters */
  float gyro_noise_var                 = 0.0f;
  float accel_noise_var                = 0.0f;
  float gyro_bias_noise_autocorr_time  = 0.0f;
  float accel_bias_noise_autocorr_time = 0.0f;
  Eigen::Quaternionf imu_orientation   = Eigen::Quaternionf::Identity();
};

class Quadrotor {
public:
  Quadrotor(quadrotor_params& params);
  ~Quadrotor();

  void update(float dt, const actuation::Acro& actuation);

  inline void get_state(State& state) const { state = *state_; };

  inline void get_imu_measurement(Eigen::Vector3f& gyro, Eigen::Vector3f& accel) const {
    imu_->get_measurement(gyro, accel);
  };

  inline void get_imu_measurement(state::Kinematics& measurement) const {
    imu_->get_measurement(measurement);
  };

  void apply_floor_force();
  void acro_to_motor_speeds(const actuation::Acro& actuation);
  void process_euler_explicit(float dt);
  void apply_floor_limit();
  void update_imu(float dt);

public:
  // Shared variables
  std::shared_ptr<Model> model_;
  std::shared_ptr<State> state_;
  std::shared_ptr<Dynamics> dynamics_;
  std::shared_ptr<FlightController> flight_controller_;
  std::shared_ptr<IMU> imu_;

private:
  actuation::MotorW actuation_motor_w_;

  float floor_height_;
  Eigen::Vector3f external_force_ = Eigen::Vector3f::Zero();
  Eigen::Vector3f floor_force_    = Eigen::Vector3f::Zero();
};  // class Quadrotor

}  // namespace quadrotor

#endif  // QUADROTOR_HPP