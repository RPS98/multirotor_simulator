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

#ifndef IMU_SIMULATOR_IMU_HPP_
#define IMU_SIMULATOR_IMU_HPP_

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <chrono>
#include <iostream>
#include <memory>
#include <random>
#include <utility>

namespace imu {

/**
 * @brief Inertial Measurement Unit parameters
 *
 * @tparam P Floating point type
 */
template <typename P = double>
struct IMUParams {
  static_assert(std::is_floating_point<P>::value,
                "MotorParams must be used with a floating-point type");

  using Precision  = P;
  using Scalar     = Precision;
  using Quaternion = Eigen::Quaternion<Precision>;

  Scalar gyro_noise_var                 = 0.0;                     // rad^2/s^2
  Scalar accel_noise_var                = 0.0;                     // m^2/s^4
  Scalar gyro_bias_noise_autocorr_time  = 0.0;                     // rad^2/s^3
  Scalar accel_bias_noise_autocorr_time = 0.0;                     // m^2/s^5
  Quaternion imu_orientation            = Quaternion::Identity();  // Quaternion

  int seed = 0;  // Seed for the random number generator
};

/**
 * @brief Inertial Measurement Unit class
 *
 * @tparam P Floating point type
 */
template <typename P = double>
class IMU {
  using Precision = P;
  using Scalar    = Precision;

  // Eigen types
  using Vector3    = Eigen::Matrix<Precision, 3, 1>;
  using Vector4    = Eigen::Matrix<Precision, 4, 1>;
  using Quaternion = Eigen::Quaternion<Precision>;

public:
  /**
   * @brief Construct a new IMU
   *
   * @param gyro_noise_var Gyroscope noise variance (rad^2/s^2)
   * @param accel_noise_var Accelerometer noise variance (m^2/s^4)
   * @param gyro_bias_noise_autocorr_time Gyroscope bias noise autocorrelation time (s)
   * @param accel_bias_noise_autocorr_time Accelerometer bias noise autocorrelation time (s)
   * @param imu_orientation IMU orientation from body frame (rad)
   * @param seed Seed for the random number generator
   */
  IMU(Scalar gyro_noise_var                 = 0.0,
      Scalar accel_noise_var                = 0.0,
      Scalar gyro_bias_noise_autocorr_time  = 0.0,
      Scalar accel_bias_noise_autocorr_time = 0.0,
      Quaternion imu_orientation            = Quaternion::Identity(),
      int seed                              = 0)
      : gyro_noise_var_(gyro_noise_var), accel_noise_var_(accel_noise_var),
        gyro_bias_noise_autocorr_time_(gyro_bias_noise_autocorr_time),
        accel_bias_noise_autocorr_time_(accel_bias_noise_autocorr_time),
        imu_orientation_(imu_orientation) {
    // Random number generator
    random_number_generator_.seed(seed);

    // Standard normal distribution
    set_bias_standard_normal_distribution(gyro_noise_var, accel_noise_var,
                                          gyro_bias_noise_autocorr_time,
                                          accel_bias_noise_autocorr_time);
  }

  /**
   * @brief Construct a new IMU
   *
   * @param params IMU parameters
   */
  explicit IMU(const IMUParams<Precision>& params)
      : IMU(params.gyro_noise_var,
            params.accel_noise_var,
            params.gyro_bias_noise_autocorr_time,
            params.accel_bias_noise_autocorr_time,
            params.imu_orientation,
            params.seed) {}

  /**
   * @brief Update the IMU state
   *
   * @param dt Delta time in seconds
   * @param specific_acceleration Total acceleration acting on the IMU in body frame. Hover value
   * is 9.81 m/s^2.
   * @param vehicle_angular_velocity Angular velocity of the vehicle in the body frame.
   */
  void update(const Scalar dt,
              const Vector3 specific_acceleration,
              const Vector3 vehicle_angular_velocity) {
    if (dt <= 0.0f) {
      return;
    }

    process_bias(dt);
    process_measurement(specific_acceleration, vehicle_angular_velocity);
    return;
  }

  /**
   * @brief Reset the IMU bias and measurement.
   */
  void reset() {
    // Reset bias
    gyro_bias_  = Vector3::Zero();
    accel_bias_ = Vector3::Zero();

    // Reset measurement
    gyro_  = Vector3::Zero();
    accel_ = Vector3::Zero();
  }

  // Setters

  /**
   * @brief Set the random seed for the random number generator.
   *
   * @param seed Random seed.
   */
  inline void set_random_seed(const int seed) { random_number_generator_.seed(seed); }

  /**
   * @brief Set the noise variances for the IMU.
   *
   * @param gyro_noise_var Gyroscope noise variance.
   * @param accel_noise_var Accelerometer noise variance.
   */
  inline void set_noise_variances(const Scalar gyro_noise_var, const Scalar accel_noise_var) {
    gyro_noise_var_  = gyro_noise_var;
    accel_noise_var_ = accel_noise_var;
  }

  /**
   * @brief Set the autocorrelation time for the IMU bias.
   *
   * @param gyro_bias_noise_autocorr_time Gyroscope bias autocorrelation time.
   * @param accel_bias_noise_autocorr_time Accelerometer bias autocorrelation time.
   */
  inline void set_bias_autocorrelation_time(const Scalar gyro_bias_noise_autocorr_time,
                                            const Scalar accel_bias_noise_autocorr_time) {
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
  inline void set_bias(const Vector3& gyro_bias,
                       const Vector3& accel_bias,
                       const Scalar gyro_bias_noise_autocorr_time,
                       const Scalar accel_bias_noise_autocorr_time) {
    gyro_bias_  = gyro_bias;
    accel_bias_ = accel_bias;
    set_bias_autocorrelation_time(gyro_bias_noise_autocorr_time, accel_bias_noise_autocorr_time);
  }

  /**
   * @brief Set the IMU orientation.
   *
   * @param imu_orientation IMU orientation with respect to the body frame.
   */
  inline void set_orientation(const Quaternion& imu_orientation) {
    imu_orientation_        = imu_orientation;
    imu_orientation_inverse = imu_orientation_.inverse();
  }

  // Getters

  /**
   * @brief Return the IMU measurement.
   *
   * @param gyro Output gyroscope measurement in body frame.
   * @param accel Output accelerometer measurement in body frame.
   */
  void get_measurement(Vector3& gyro, Vector3& accel) const {
    gyro  = gyro_;
    accel = accel_;
  }

  /**
   * @brief Return the IMU measurement.
   *
   * @return const std::pair<Vector3, Vector3>& Gyroscope and accelerometer measurement in body
   * frame.
   */
  inline const std::pair<Vector3, Vector3>& get_measurement() const {
    return std::make_pair(gyro_, accel_);
  }

  /**
   * @brief Get the measurement gyro in body frame.
   *
   * @param gyro Output gyroscope measurement in body frame.
   */
  inline void get_measurement_gyro(Vector3& gyro) const { gyro = gyro_; }

  /**
   * @brief Get the measurement gyro in body frame (const).
   *
   * @return const Vector3& Gyroscope measurement in body frame.
   */
  inline const Vector3& get_measurement_gyro() const { return gyro_; }

  /**
   * @brief Get the measurement accel
   *
   * @param accel Output accelerometer measurement in body frame.
   */
  void get_measurement_accel(Vector3& accel) const { accel = accel_; }

  /**
   * @brief Get the measurement accel in body frame (const).
   *
   * @return const Vector3& Accelerometer measurement in body frame.
   */
  const Vector3& get_measurement_accel() const { return accel_; }

  /**
   * @brief Get IMU bias.
   *
   * @param gyro_bias Gyroscope bias.
   * @param accel_bias Accelerometer bias.
   */
  void get_bias(Vector3& gyro_bias, Vector3& accel_bias) const {
    gyro_bias  = gyro_bias_;
    accel_bias = accel_bias_;
  }

  /**
   * @brief Get IMU variance.
   *
   * @param gyro_bias_var Gyroscope bias variance.
   * @param accel_bias_var Accelerometer bias variance.
   */
  void get_variances(Scalar& gyro_bias_var, Scalar& accel_bias_var) const {
    gyro_bias_var  = gyro_noise_var_;
    accel_bias_var = accel_noise_var_;
  }

protected:
  // Noise properties
  std::default_random_engine random_number_generator_;  // Random number generator
  std::normal_distribution<P> standard_normal_distribution_ =
      std::normal_distribution<P>(0.0, 1.0);  // Standard normal distribution

  // Noise variance
  Scalar gyro_noise_var_  = 0.0;  // rad^2/s^2
  Scalar accel_noise_var_ = 0.0;  // m^2/s^4

  // Bias noise autocorrelation time
  Scalar gyro_bias_noise_autocorr_time_  = 0.0;  // rad^2/s^3
  Scalar accel_bias_noise_autocorr_time_ = 0.0;  // m^2/s^5

  // IMU frame
  Quaternion imu_orientation_        = Quaternion::Identity();      // Quaternion
  Quaternion imu_orientation_inverse = imu_orientation_.inverse();  // Quaternion

  // Bias
  Vector3 gyro_bias_  = Vector3::Zero();  // rad/s
  Vector3 accel_bias_ = Vector3::Zero();  // m/s^2

  // Measurement in body frame
  Vector3 gyro_  = Vector3::Zero();  // rad/s
  Vector3 accel_ = Vector3::Zero();  // m/s^2

protected:
  /**
   * @brief Set the IMU bias using standard normal distribution.
   *
   * @param gyro_noise_var Gyroscope bias variance.
   * @param accel_noise_var Accelerometer bias variance.
   * @param gyro_bias_noise_autocorr_time Gyroscope bias autocorrelation time.
   * @param accel_bias_noise_autocorr_time Accelerometer bias autocorrelation time.
   */
  void set_bias_standard_normal_distribution(const Scalar gyro_noise_var,
                                             const Scalar accel_noise_var,
                                             const Scalar gyro_bias_noise_autocorr_time,
                                             const Scalar accel_bias_noise_autocorr_time) {
    gyro_bias_ =
        Vector3(sqrt(gyro_noise_var) * standard_normal_distribution_(random_number_generator_),
                sqrt(gyro_noise_var) * standard_normal_distribution_(random_number_generator_),
                sqrt(gyro_noise_var) * standard_normal_distribution_(random_number_generator_));

    accel_bias_ =
        Vector3(sqrt(accel_noise_var) * standard_normal_distribution_(random_number_generator_),
                sqrt(accel_noise_var) * standard_normal_distribution_(random_number_generator_),
                sqrt(accel_noise_var) * standard_normal_distribution_(random_number_generator_));

    set_bias_autocorrelation_time(gyro_bias_noise_autocorr_time, accel_bias_noise_autocorr_time);
  }

  /**
   * @brief Process the IMU bias.
   *
   * @param dt Time step.
   */
  void process_bias(const Scalar dt) {
    // Gyroscope derivative
    Vector3 gyro_bias_derivative =
        Vector3(sqrt(gyro_bias_noise_autocorr_time_ / dt) *
                    standard_normal_distribution_(random_number_generator_),
                sqrt(gyro_bias_noise_autocorr_time_ / dt) *
                    standard_normal_distribution_(random_number_generator_),
                sqrt(gyro_bias_noise_autocorr_time_ / dt) *
                    standard_normal_distribution_(random_number_generator_));

    // Accelerometer derivative
    Vector3 accel_bias_derivative =
        Vector3(sqrt(accel_bias_noise_autocorr_time_ / dt) *
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
   * @param specific_acceleration Total acceleration acting on the IMU in body frame.
   * @param vehicle_angular_velocity Angular velocity of the vehicle in the body frame.
   */
  void process_measurement(const Vector3& specific_acceleration,
                           const Vector3& vehicle_angular_velocity) {
    Vector3 gyro_noise =
        Vector3(sqrt(gyro_noise_var_) * standard_normal_distribution_(random_number_generator_),
                sqrt(gyro_noise_var_) * standard_normal_distribution_(random_number_generator_),
                sqrt(gyro_noise_var_) * standard_normal_distribution_(random_number_generator_));

    Vector3 accel_noise =
        Vector3(sqrt(accel_noise_var_) * standard_normal_distribution_(random_number_generator_),
                sqrt(accel_noise_var_) * standard_normal_distribution_(random_number_generator_),
                sqrt(accel_noise_var_) * standard_normal_distribution_(random_number_generator_));

    // Rotate specific force and angular velocity to IMU frame
    gyro_  = imu_orientation_inverse * vehicle_angular_velocity + gyro_bias_ + gyro_noise;
    accel_ = imu_orientation_inverse * specific_acceleration + accel_bias_ + accel_noise;
  }
};  // class IMU

}  // namespace imu

#endif  // IMU_SIMULATOR_IMU_HPP_
