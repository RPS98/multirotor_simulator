/*!*******************************************************************************************
 *  \file       imu.hpp
 *  \brief      Inertial Measurement Unit class definition
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

#ifndef IMU_HPP
#define IMU_HPP

#include <chrono>
#include <iostream>
#include <memory>
#include <random>

#include "common/actuation.hpp"
#include "common/model.hpp"
#include "common/state.hpp"

namespace quadrotor {

class IMU {
public:
  IMU(float gyro_noise_var,
      float accel_noise_var,
      float gyro_bias_noise_autocorr_time,
      float accel_bias_noise_autocorr_time,
      Eigen::Quaternionf imu_orientation);

  ~IMU();

public:
  // Public methods
  void process_bias(const float dt);

  void get_measurement(const Eigen::Vector3f specific_force,
                       const Eigen::Vector3f vehicle_angular_velocity,
                       Eigen::Vector3f& gyro,
                       Eigen::Vector3f& accel);

  // Setters
  void set_random_seed(const int seed);

  void set_noise_variances(const float gyro_noise_var, const float accel_noise_var);

  void set_bias_autocorrelation_time(const float gyro_bias_noise_autocorr_time,
                                     const float accel_bias_noise_autocorr_time);

  void set_bias(const Eigen::Vector3f& gyro_bias,
                const Eigen::Vector3f& accel_bias,
                const float gyro_bias_noise_autocorr_time,
                const float accel_bias_noise_autocorr_time);

  void set_bias(const float gyro_bias_var,
                const float accel_bias_var,
                const float gyro_bias_noise_autocorr_time,
                const float accel_bias_noise_autocorr_time);

  void set_bias(const float gyro_bias_var, const float accel_bias_var);

  void set_orientation(const Eigen::Quaternionf imu_orientation);

  // Getters
  void get_bias(Eigen::Vector3f& gyro_bias, Eigen::Vector3f& accel_bias) const;

  void get_variances(float& gyro_bias_var, float& accel_bias_var) const;

private:
  // Noise properties
  std::default_random_engine random_number_generator_;  // Random number generator
  std::normal_distribution<float> standard_normal_distribution_ =
      std::normal_distribution<float>(0.0f, 1.0f);  // Standard normal distribution

  // Noise variance
  float gyro_noise_var_  = 0.0f;  // rad^2/s^2
  float accel_noise_var_ = 0.0f;  // m^2/s^4

  // Bias noise autocorrelation time
  float gyro_bias_noise_autocorr_time_  = 0.0f;  // rad^2/s^3
  float accel_bias_noise_autocorr_time_ = 0.0f;  // m^2/s^5

  // IMU frame
  Eigen::Quaternionf imu_orientation_ = Eigen::Quaternionf::Identity();  // Quaternion

  // Bias
  Eigen::Vector3f gyro_bias_  = Eigen::Vector3f::Zero();  // rad/s
  Eigen::Vector3f accel_bias_ = Eigen::Vector3f::Zero();  // m/s^2

};  // class IMU

}  // namespace quadrotor

#endif  // IMU_HPP