/*!*******************************************************************************************
 *  \file       model.hpp
 *  \brief      Quadrotor model class definition
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

#ifndef MODEL_HPP
#define MODEL_HPP

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <chrono>
#include <random>
#include <vector>

namespace quadrotor {

class Model {
public:
  Model(float motor_thrust_coefficient,
        float motor_torque_coefficient,
        float motor_dx,
        float motor_dy,
        float motor_min_speed,
        float motor_max_speed,
        float motor_time_constant,
        float motor_rotational_inertia,
        float vehicle_mass,
        const Eigen::Matrix3f& vehicle_inertia,
        float vehicle_drag_coefficient              = 0.0f,
        float vehicle_aero_moment_coefficient       = 0.0f,
        const Eigen::Vector3f& gravity              = Eigen::Vector3f(0.0f, 0.0f, -9.81f),
        float moment_process_noise_auto_correlation = 0.0f,
        float force_process_noise_auto_correlation  = 0.0f);

  ~Model();

public:
  float mass;

  // Quadrotor frame
  Eigen::Matrix<float, 3, 4> motors_frame_thrust_coefficient_matrix;
  Eigen::Matrix<float, 3, 4> motors_frame_torque_coefficient_matrix;
  Eigen::Matrix<float, 3, 4> motors_frame_inertia_matrix;

  // Motors properties
  float motor_thrust_coefficient;
  float motor_torque_coefficient;
  float motor_min_speed = 0.0f;
  float motor_max_speed;
  float motor_time_constant;
  float motor_rotational_inertia;

  // Vehicle properties
  float vehicle_mass;                            // kg
  float vehicle_drag_coefficient        = 0.0f;  // N/(m/s)
  float vehicle_aero_moment_coefficient = 0.0f;  // N · m/(rad/s)^2
  Eigen::Matrix3f vehicle_inertia;               // kg · m^2

  // Environment properties
  Eigen::Vector3f gravity = Eigen::Vector3f(0.0f, 0.0f, -9.81f);  // m/s^2

  // Noise properties
  std::default_random_engine random_number_generator;  // Random number generator
  std::normal_distribution<float> standard_normal_distribution =
      std::normal_distribution<float>(0.0f, 1.0f);     // Standard normal distribution
  float moment_process_noise_auto_correlation = 0.0f;  // (N · m)^2s
  float force_process_noise_auto_correlation  = 0.0f;  // N^2s
  Eigen::Vector3f stoch_force                 = Eigen::Vector3f::Zero();  // N
  Eigen::Vector3f stoch_moment                = Eigen::Vector3f::Zero();  // N · m

public:
  void set_stochastic_noise(const float dt);
  Eigen::Vector3f get_stochastic_force() const;
  Eigen::Vector3f get_stochastic_moment() const;

  Eigen::Vector3f get_gravity_force() const;

  Eigen::Vector3f get_thrust_force(const Eigen::Vector4f& motors_angular_velocity) const;
  static Eigen::Vector3f get_thrust_force(const float motors_thrust_coefficient,
                                          const Eigen::Vector4f& motors_angular_velocity);

  Eigen::Vector3f get_drag_force(const Eigen::Vector3f& vehicle_linear_velocity) const;
  static Eigen::Vector3f get_drag_force(const float vehicle_drag_coefficient,
                                        const Eigen::Vector3f& vehicle_linear_velocity);

  Eigen::Vector3f get_aerodynamic_moment(const Eigen::Vector3f& vehicle_angular_velocity) const;
  static Eigen::Vector3f get_aerodynamic_moment(const float vehicle_aerodynamic_coefficient,
                                                const Eigen::Vector3f& vehicle_angular_velocity);

  Eigen::Vector3f get_motor_torque(const Eigen::Vector4f& motors_angular_velocity,
                                   const Eigen::Vector4f& motors_angular_acceleration) const;

  // Getters
  inline float get_mass() const { return this->mass; }

  inline Eigen::Matrix3f get_vehicle_inertia() const { return this->vehicle_inertia; }

  inline float get_motors_thrust_coefficient() const { return this->motor_thrust_coefficient; }

  inline Eigen::Matrix<float, 3, 4> get_motors_frame_thrust_coefficient_matrix() const {
    return this->motors_frame_thrust_coefficient_matrix;
  };

  inline Eigen::Matrix<float, 3, 4> get_motors_frame_torque_coefficient_matrix() const {
    return this->motors_frame_torque_coefficient_matrix;
  };

  inline Eigen::Matrix<float, 3, 4> get_motors_frame_inertia_matrix() const {
    return this->motors_frame_inertia_matrix;
  };
};  // class Model

}  // namespace quadrotor

#endif  // MODEL_HPP