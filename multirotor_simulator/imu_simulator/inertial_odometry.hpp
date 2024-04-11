/*!*******************************************************************************************
 *  \file       inertial_odometry.hpp
 *  \brief      Inertial Odometry class definition
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

#ifndef IMU_SIMULATOR_INERTIAL_ODOMETRY_HPP_
#define IMU_SIMULATOR_INERTIAL_ODOMETRY_HPP_

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <chrono>
#include <iostream>
#include <memory>
#include <random>

#include <imu_simulator/imu.hpp>

namespace imu {

/**
 * @brief Inertial Odometry parameters
 *
 * @tparam P Floating point type
 */
template <typename P = double>
struct InertialOdometryParams {
  static_assert(std::is_floating_point<P>::value,
                "MotorParams must be used with a floating-point type");

  using Scalar = P;

  // Low pass filter alpha. 1: no filter, 0: no update
  Scalar alpha = 1.0;

  // Initial orientation
  Eigen::Quaternion<P> initial_world_orientation = Eigen::Quaternion<P>::Identity();
};

/**
 * @brief Inertial Odometry class
 *
 * @tparam P Floating point type
 */
template <typename P = double>
class InertialOdometry {
  using Precision = P;
  using Scalar    = Precision;

  // Eigen types
  using Vector3    = Eigen::Matrix<Precision, 3, 1>;
  using Vector4    = Eigen::Matrix<Precision, 4, 1>;
  using Quaternion = Eigen::Quaternion<Precision>;

public:
  /**
   * @brief Construct a new Inertial Odometry
   *
   * @param alpha Filter parameter (0 < alpha < 1, 1: no filter)
   * @param initial_world_orientation Initial orientation in world frame
   */
  InertialOdometry(Scalar alpha                         = 1.0,
                   Quaternion initial_world_orientation = Quaternion::Identity())
      : alpha_(alpha), initial_world_orientation_(initial_world_orientation) {
    assert(alpha_ > 0 && alpha_ <= 1);
    reset();
  }

  /**
   * @brief Construct a new Inertial Odometry
   *
   * @param params Inertial Odometry parameters
   */
  explicit InertialOdometry(const InertialOdometryParams<Precision>& params)
      : InertialOdometry(params.alpha, params.initial_world_orientation) {}

  ~InertialOdometry() {}

  /**
   * @brief Set the initial orientation in world frame
   *
   * @param initial_world_orientation Initial orientation in world frame
   */
  void set_initial_orientation(const Quaternion& initial_world_orientation) {
    initial_world_orientation_ = initial_world_orientation;
    orientation_               = initial_world_orientation;
  }

  /**
   * @brief Reset the inertial odometry
   *
   */
  void reset() {
    gyro_filtered_       = Vector3::Zero();
    accel_filtered_      = Vector3::Zero();
    linear_acceleration_ = Vector3::Zero();
    linear_velocity_     = Vector3::Zero();
    position_            = Vector3::Zero();
    angular_velocity_    = Vector3::Zero();
    orientation_         = initial_world_orientation_;
  }
  /**
   * @brief Update the inertial odometry with a new measurement
   *
   * @param gyro Gyro measurement in body frame (rad/s)
   * @param accel Accelerometer measurement in body frame (m/s^2)
   * @param dt Time step (s)
   */
  void update(const Vector3& gyro, const Vector3& accel, const Scalar dt) {
    filter_measurement(gyro, accel);
    integrate_measurement(dt);
  }

  /**
   * @brief Get the measurement
   *
   * @param position Position in world frame (m)
   * @param orientation Orientation in world frame (rad)
   * @param linear_velocity Linear velocity in world frame (m/s)
   * @param angular_velocity Angular velocity in body frame (rad/s)
   * @param linear_acceleration Linear acceleration in world frame (m/s^2)
   */
  void get_measurement(Vector3& position,
                       Quaternion& orientation,
                       Vector3& linear_velocity,
                       Vector3& angular_velocity,
                       Vector3& linear_acceleration) const {
    position            = position_;
    orientation         = orientation_;
    linear_velocity     = linear_velocity_;
    angular_velocity    = angular_velocity_;
    linear_acceleration = linear_acceleration_;
  }

  /**
   * @brief Get the position
   *
   * @return Vector3 Position in world frame (m)
   */
  inline Vector3 get_position() const { return position_; }

  /**
   * @brief Get the orientation
   *
   * @return Quaternion Orientation in world frame (rad)
   */
  inline Quaternion get_orientation() const { return orientation_; }

  /**
   * @brief Get the linear velocity
   *
   * @return Vector3 Linear velocity in world frame (m/s)
   */
  inline Vector3 get_linear_velocity() const { return linear_velocity_; }

  /**
   * @brief Get the angular velocity
   *
   * @return Vector3 Angular velocity in body frame (rad/s)
   */
  inline Vector3 get_angular_velocity() const { return angular_velocity_; }

  /**
   * @brief Get the linear acceleration
   *
   * @return Vector3 Linear acceleration in world frame (m/s^2)
   */
  inline Vector3 get_linear_acceleration() const { return linear_acceleration_; }

protected:
  // Filter
  Scalar alpha_                         = 0.1;
  Quaternion initial_world_orientation_ = Quaternion::Identity();

  // Measurement in body frame
  Vector3 gyro_filtered_  = Vector3::Zero();  // rad/s
  Vector3 accel_filtered_ = Vector3::Zero();  // m/s^2

  // Measurement in world frame
  Vector3 position_            = Vector3::Zero();         // m
  Quaternion orientation_      = Quaternion::Identity();  // Quaternion
  Vector3 linear_velocity_     = Vector3::Zero();         // m/s
  Vector3 angular_velocity_    = Vector3::Zero();         // rad/s
  Vector3 linear_acceleration_ = Vector3::Zero();         // m/s^2

protected:
  /**
   * @brief Filter the IMU measurement using a low-pass filter.
   *
   * @param gyro Angular velocity in body frame (rad/s)
   * @param accel Linear acceleration in body frame (m/s^2)
   */
  void filter_measurement(const Vector3& gyro, const Vector3& accel) {
    gyro_filtered_  = alpha_ * gyro + (1.0 - alpha_) * gyro_filtered_;
    accel_filtered_ = alpha_ * accel + (1.0 - alpha_) * accel_filtered_;
  }

  /**
   * @brief Integrate the IMU measurement using Euler integration.
   *
   * @param dt Time step (s)
   */
  void integrate_measurement(const Scalar dt) {
    // Angular velocity in body frame
    // gyro_

    // Linear acceleration in body frame
    // accel_

    // Angular velocity in body frame
    angular_velocity_ = gyro_filtered_;

    // Linear acceleration in world frame
    linear_acceleration_ = orientation_.inverse() * accel_filtered_;

    // Integrate angular velocity to get orientation quaternion
    orientation_ = get_quaternion_integrate(orientation_, angular_velocity_, dt);

    // Integrate linear acceleration to get linear velocity
    linear_velocity_ += linear_acceleration_ * dt;

    // Integrate linear velocity to get position
    position_ += linear_velocity_ * dt;
  }

  /**
   * @brief Compute the quaternion derivative.
   *
   * This function computes the quaternion derivative using Hamiltonian product:
   * q_dot = 0.5 * q * omega_q
   *
   * @param q The input quaternion
   * @param omega The angular velocity vector used for integration
   * @return Vector4 The quaternion derivative
   */
  static Vector4 get_quaternion_derivative(const Quaternion& q, const Vector3& omega) {
    // Convert the angular velocity vector to a quaternion
    Quaternion omega_q;
    omega_q.w()   = 0;
    omega_q.vec() = omega;

    // Convert the input quaternion to a quaternion with the same scalar type as the output
    // quaternion
    Quaternion q_q;
    q_q.w()   = q.w();
    q_q.vec() = q.vec();

    // Calculate the quaternion derivative using Hamiltonian product: q_dot = 0.5 * q * omega_q
    return static_cast<Precision>(0.5) * (q_q * omega_q).coeffs();
  }

  /**
   * @brief Integrate a quaternion
   *
   * This function integrates a quaternion using Euler integration:
   * q_{k+1} = q_k + q_dot * dt
   *
   * @param q The input quaternion
   * @param omega The angular velocity vector used for integration
   * @param dt The integration time step
   * @return Quaternion The integrated quaternion
   */
  static Quaternion get_quaternion_integrate(const Quaternion& q,
                                             const Vector3& omega,
                                             const Scalar dt) {
    // Calculate the quaternion derivative using the get_quaternion_derivative function
    Vector4 q_dot = get_quaternion_derivative(q, omega);

    // Integrate the quaternion using euler integration
    Quaternion q_integrated;
    q_integrated.coeffs() = q.coeffs() + q_dot * dt;

    // Normalize the resulting quaternion to ensure it remains unit length
    q_integrated.normalize();

    return q_integrated;
  }

private:
  template <typename T = double>
  static void quaternion_to_Euler(const Eigen::Quaternion<T>& _quaternion,
                                  T& roll,
                                  T& pitch,
                                  T& yaw) {
    // roll (x-axis rotation)
    Eigen::Quaternion<T> quaternion = _quaternion.normalized();

    T sinr = 2.0 * (quaternion.w() * quaternion.x() + quaternion.y() * quaternion.z());
    T cosr = 1.0 - 2.0 * (quaternion.x() * quaternion.x() + quaternion.y() * quaternion.y());
    roll   = std::atan2(sinr, cosr);

    // pitch (y-axis rotation)
    T sinp = 2.0 * (quaternion.w() * quaternion.y() - quaternion.z() * quaternion.x());
    if (std::abs(sinp) >= 1.0)
      pitch = std::copysign(M_PI / 2, sinp);  // use 90 degrees if out of range
    else
      pitch = std::asin(sinp);

    // yaw (z-axis rotation)
    T siny = 2.0 * (quaternion.w() * quaternion.z() + quaternion.x() * quaternion.y());
    T cosy = 1.0 - 2.0 * (quaternion.y() * quaternion.y() + quaternion.z() * quaternion.z());
    yaw    = std::atan2(siny, cosy);
  }

public:
  friend std::ostream& operator<<(std::ostream& os, const InertialOdometry<P>& inertial_odometry) {
    Vector3 euler_angles;
    inertial_odometry.quaternion_to_Euler(inertial_odometry.orientation_, euler_angles.x(),
                                          euler_angles.y(), euler_angles.z());
    // x,y,z,roll,pitch,yaw,vx,vy,vz,wx,wy,wz,ax,ay,az
    os << inertial_odometry.position_.x() << "," << inertial_odometry.position_.y() << ","
       << inertial_odometry.position_.z() << "," << euler_angles.x() << "," << euler_angles.y()
       << "," << euler_angles.z() << "," << inertial_odometry.linear_velocity_.x() << ","
       << inertial_odometry.linear_velocity_.y() << "," << inertial_odometry.linear_velocity_.z()
       << "," << inertial_odometry.angular_velocity_.x() << ","
       << inertial_odometry.angular_velocity_.y() << "," << inertial_odometry.angular_velocity_.z()
       << "," << inertial_odometry.linear_acceleration_.x() << ","
       << inertial_odometry.linear_acceleration_.y() << ","
       << inertial_odometry.linear_acceleration_.z();
    return os;
  }

  friend std::ostream& print_state(std::ostream& os, const InertialOdometry<P>& inertial_odometry) {
    Vector3 euler_angles;
    inertial_odometry.quaternion_to_Euler(inertial_odometry.orientation_, euler_angles.x(),
                                          euler_angles.y(), euler_angles.z());
    // x,y,z,roll,pitch,yaw,vx,vy,vz,wx,wy,wz,ax,ay,az
    os << "position_x: " << inertial_odometry.position_.x() << ", "
       << "position_y: " << inertial_odometry.position_.y() << ", "
       << "position_z: " << inertial_odometry.position_.z() << ", "
       << "orientation_roll: " << euler_angles.x() << ", "
       << "orientation_pitch: " << euler_angles.y() << ", "
       << "orientation_yaw: " << euler_angles.z() << ", "
       << "linear_velocity_x: " << inertial_odometry.linear_velocity_.x() << ", "
       << "linear_velocity_y: " << inertial_odometry.linear_velocity_.y() << ", "
       << "linear_velocity_z: " << inertial_odometry.linear_velocity_.z() << ", "
       << "angular_velocity_x: " << inertial_odometry.angular_velocity_.x() << ", "
       << "angular_velocity_y: " << inertial_odometry.angular_velocity_.y() << ", "
       << "angular_velocity_z: " << inertial_odometry.angular_velocity_.z() << ", "
       << "linear_acceleration_x: " << inertial_odometry.linear_acceleration_.x() << ", "
       << "linear_acceleration_y: " << inertial_odometry.linear_acceleration_.y() << ", "
       << "linear_acceleration_z: " << inertial_odometry.linear_acceleration_.z();
    return os;
  }
};  // class InertialOdometry

}  // namespace imu

#endif  // IMU_SIMULATOR_INERTIAL_ODOMETRY_HPP_
