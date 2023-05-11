/*!*******************************************************************************************
 *  \file       imu.cpp
 *  \brief      Inertial Measurement Unit implementation file.
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

#include "quadrotor_model/imu/imu.hpp"

using namespace quadrotor;

/**
 * @brief Construct a new IMU::IMU object
 *
 * @param gyro_noise_var Gyroscope noise variance.
 * @param accel_noise_var Accelerometer noise variance.
 * @param gyro_bias_noise_autocorr_time Gyroscope bias noise autocorrelation time.
 * @param accel_bias_noise_autocorr_time Accelerometer bias noise autocorrelation time.
 * @param imu_orientation IMU orientation with respect to the body frame.
 */
IMU::IMU(float gyro_noise_var,
         float accel_noise_var,
         float gyro_bias_noise_autocorr_time,
         float accel_bias_noise_autocorr_time,
         Eigen::Quaternionf initial_world_orientation,
         Eigen::Quaternionf imu_orientation = Eigen::Quaternionf::Identity())
    : gyro_noise_var_(gyro_noise_var), accel_noise_var_(accel_noise_var),
      gyro_bias_noise_autocorr_time_(gyro_bias_noise_autocorr_time),
      accel_bias_noise_autocorr_time_(accel_bias_noise_autocorr_time),
      orientation_(initial_world_orientation), imu_orientation_(imu_orientation) {
  // Random number generator
  random_number_generator_.seed(std::chrono::system_clock::now().time_since_epoch().count());

  // Standard normal distribution
  set_bias(gyro_noise_var, accel_noise_var, gyro_bias_noise_autocorr_time,
           accel_bias_noise_autocorr_time);
}

IMU::~IMU() {
  // TODO
}

/**
 * @brief Set the random seed for the random number generator.
 *
 * @param seed Seed for the random number generator.
 */
void IMU::set_random_seed(const int seed) { random_number_generator_.seed(seed); }

/**
 * @brief Set the noise variances for the IMU.
 *
 * @param gyro_noise_var Gyroscope noise variance.
 * @param accel_noise_var Accelerometer noise variance.
 */
inline void IMU::set_noise_variances(const float gyro_noise_var, const float accel_noise_var) {
  gyro_noise_var_  = gyro_noise_var;
  accel_noise_var_ = accel_noise_var;
}

/**
 * @brief Set the autocorrelation time for the IMU bias.
 *
 * @param gyro_bias_noise_autocorr_time Gyroscope bias autocorrelation time.
 * @param accel_bias_noise_autocorr_time Accelerometer bias autocorrelation time.
 */
inline void IMU::set_bias_autocorrelation_time(const float gyro_bias_noise_autocorr_time,
                                               const float accel_bias_noise_autocorr_time) {
  gyro_bias_noise_autocorr_time_  = gyro_bias_noise_autocorr_time;
  accel_bias_noise_autocorr_time_ = accel_bias_noise_autocorr_time;
}

/**
 * @brief Set the IMU bias.
 *
 * @param gyro_bias Gyroscope bias.
 * @param accel_bias Accelerometer bias.
 * @param gyro_bias_noise_autocorr_time Gyroscope bias autocorrelation time.
 * @param accel_bias_noise_autocorr_time Accelerometer bias autocorrelation time.
 */
inline void IMU::set_bias(const Eigen::Vector3f& gyro_bias,
                          const Eigen::Vector3f& accel_bias,
                          const float gyro_bias_noise_autocorr_time,
                          const float accel_bias_noise_autocorr_time) {
  gyro_bias_  = gyro_bias;
  accel_bias_ = accel_bias;
  set_bias_autocorrelation_time(gyro_bias_noise_autocorr_time, accel_bias_noise_autocorr_time);
}

/**
 * @brief Set the IMU bias.
 *
 * @param gyro_noise_var Gyroscope bias variance.
 * @param accel_noise_var Accelerometer bias variance.
 * @param gyro_bias_noise_autocorr_time Gyroscope bias autocorrelation time.
 * @param accel_bias_noise_autocorr_time Accelerometer bias autocorrelation time.
 */
void IMU::set_bias(const float gyro_noise_var,
                   const float accel_noise_var,
                   const float gyro_bias_noise_autocorr_time,
                   const float accel_bias_noise_autocorr_time) {
  gyro_bias_ = Eigen::Vector3f(
      sqrt(gyro_noise_var) * standard_normal_distribution_(random_number_generator_),
      sqrt(gyro_noise_var) * standard_normal_distribution_(random_number_generator_),
      sqrt(gyro_noise_var) * standard_normal_distribution_(random_number_generator_));

  accel_bias_ = Eigen::Vector3f(
      sqrt(accel_noise_var) * standard_normal_distribution_(random_number_generator_),
      sqrt(accel_noise_var) * standard_normal_distribution_(random_number_generator_),
      sqrt(accel_noise_var) * standard_normal_distribution_(random_number_generator_));

  set_bias_autocorrelation_time(gyro_bias_noise_autocorr_time, accel_bias_noise_autocorr_time);
}

/**
 * @brief Set the IMU bias.
 *
 * @param gyro_bias_var Gyroscope bias variance.
 * @param accel_bias_var Accelerometer bias variance.
 */
inline void IMU::set_bias(const float gyro_bias_var, const float accel_bias_var) {
  set_bias(gyro_bias_var, accel_bias_var, gyro_bias_noise_autocorr_time_,
           accel_bias_noise_autocorr_time_);
}

/**
 * @brief Set the IMU orientation.
 *
 * @param imu_orientation IMU orientation with respect to the body frame.
 */
void IMU::set_orientation(const Eigen::Quaternionf imu_orientation) {
  imu_orientation_ = imu_orientation;
}

/**
 * @brief Get IMU bias.
 *
 * @param gyro_bias Gyroscope bias.
 * @param accel_bias Accelerometer bias.
 */
void IMU::get_bias(Eigen::Vector3f& gyro_bias, Eigen::Vector3f& accel_bias) const {
  gyro_bias  = gyro_bias_;
  accel_bias = accel_bias_;
}

/**
 * @brief Get IMU variance.
 *
 * @param gyro_bias_var
 * @param accel_bias_var
 */
void IMU::get_variances(float& gyro_bias_var, float& accel_bias_var) const {
  gyro_bias_var  = gyro_noise_var_;
  accel_bias_var = accel_noise_var_;
}

/**
 * @brief Return the IMU measurement.
 *
 * @param gyro Output gyroscope measurement in body frame.
 * @param accel Output accelerometer measurement in body frame.
 */
void IMU::get_measurement(Eigen::Vector3f& gyro, Eigen::Vector3f& accel) const {
  gyro  = gyro_;
  accel = accel_;
}

/**
 * @brief Return the IMU measurement.
 *
 * @param position Output position integrated from the IMU in the world frame.
 * @param orientation Output orientation integrated from the IMU in the world frame.
 * @param linear_velocity Output linear velocity integrated from the IMU in the world frame.
 * @param angular_velocity Output angular velocity integrated from the IMU in the world frame.
 * @param linear_acceleration Output linear acceleration integrated from the IMU in the world
 * frame.
 */
void IMU::get_measurement(Eigen::Vector3f& position,
                          Eigen::Quaternionf& orientation,
                          Eigen::Vector3f& linear_velocity,
                          Eigen::Vector3f& angular_velocity,
                          Eigen::Vector3f& linear_acceleration) const {
  position            = position_;
  orientation         = orientation_;
  linear_velocity     = linear_velocity_;
  angular_velocity    = angular_velocity_;
  linear_acceleration = linear_acceleration_;
}

/**
 * @brief Return the IMU measurement.
 *
 * @param measurement Output state::Kinematics measurement integrated from the IMU in the world
 * frame.
 */
void IMU::get_measurement(state::Kinematics& measurement) const {
  measurement.position            = position_;
  measurement.orientation         = orientation_;
  measurement.linear_velocity     = linear_velocity_;
  measurement.angular_velocity    = angular_velocity_;
  measurement.linear_acceleration = linear_acceleration_;
}

/**
 * @brief Process the IMU bias.
 *
 * @param dt Time step.
 */
void IMU::process_bias(const float dt) {
  // Gyroscope derivative
  Eigen::Vector3f gyro_bias_derivative =
      Eigen::Vector3f(sqrt(gyro_bias_noise_autocorr_time_ / dt) *
                          standard_normal_distribution_(random_number_generator_),
                      sqrt(gyro_bias_noise_autocorr_time_ / dt) *
                          standard_normal_distribution_(random_number_generator_),
                      sqrt(gyro_bias_noise_autocorr_time_ / dt) *
                          standard_normal_distribution_(random_number_generator_));

  // Accelerometer derivative
  Eigen::Vector3f accel_bias_derivative =
      Eigen::Vector3f(sqrt(accel_bias_noise_autocorr_time_ / dt) *
                          standard_normal_distribution_(random_number_generator_),
                      sqrt(accel_bias_noise_autocorr_time_ / dt) *
                          standard_normal_distribution_(random_number_generator_),
                      sqrt(accel_bias_noise_autocorr_time_ / dt) *
                          standard_normal_distribution_(random_number_generator_));

  // Update bias
  gyro_bias_ += gyro_bias_derivative * dt;
  accel_bias_ += accel_bias_derivative * dt;
}

/**
 * @brief Update the IMU measurement.
 *
 * @param specific_force Total force acting on the IMU without gravity in body frame.
 * @param vehicle_angular_velocity Angular velocity of the vehicle in the body frame.
 */
void IMU::process_measurement(const Eigen::Vector3f specific_force,
                              const Eigen::Vector3f vehicle_angular_velocity) {
  Eigen::Vector3f gyro_noise = Eigen::Vector3f(
      sqrt(gyro_noise_var_) * standard_normal_distribution_(random_number_generator_),
      sqrt(gyro_noise_var_) * standard_normal_distribution_(random_number_generator_),
      sqrt(gyro_noise_var_) * standard_normal_distribution_(random_number_generator_));

  Eigen::Vector3f accel_noise = Eigen::Vector3f(
      sqrt(accel_noise_var_) * standard_normal_distribution_(random_number_generator_),
      sqrt(accel_noise_var_) * standard_normal_distribution_(random_number_generator_),
      sqrt(accel_noise_var_) * standard_normal_distribution_(random_number_generator_));

  // Rotate specific force and angular velocity to IMU frame
  gyro_  = imu_orientation_.inverse() * vehicle_angular_velocity + gyro_bias_ + gyro_noise;
  accel_ = imu_orientation_.inverse() * specific_force + accel_bias_ + accel_noise;
}

/**
 * @brief Integrate the IMU measurement using Euler integration.
 *
 * @param dt Time step in seconds.
 */
void IMU::integrate_measurement(const float dt) {
  // Angular velocity in body frame
  // gyro_

  // Linear acceleration in body frame
  // accel_

  // Angular velocity in world frame
  angular_velocity_ = orientation_ * gyro_;

  // Linear acceleration in world frame
  linear_acceleration_ = orientation_ * accel_;

  // Integrate angular velocity to get orientation quaternion
  orientation_ = utils::get_quaternion_integrate(orientation_, gyro_, dt);

  // Integrate linear acceleration to get linear velocity
  linear_velocity_ += accel_ * dt;

  // Integrate linear velocity to get position
  position_ += linear_velocity_ * dt;
}

/**
 * @brief Update the IMU state
 *
 * @param dt Delta time in seconds
 * @param specific_force Total force acting on the IMU without gravity in body frame.
 * @param vehicle_angular_velocity Angular velocity of the vehicle in the body frame.
 */
void IMU::update(const float dt,
                 const Eigen::Vector3f specific_force,
                 const Eigen::Vector3f vehicle_angular_velocity) {
  process_bias(dt);
  process_measurement(specific_force, vehicle_angular_velocity);
  integrate_measurement(dt);
}