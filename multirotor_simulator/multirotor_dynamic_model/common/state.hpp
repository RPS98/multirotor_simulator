/*!*******************************************************************************************
 *  \file       state.hpp
 *  \brief      Multirotor State definition
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

#ifndef MULTIROTOR_DYNAMIC_MODEL_COMMON_STATE_HPP_
#define MULTIROTOR_DYNAMIC_MODEL_COMMON_STATE_HPP_

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <iostream>

namespace multirotor {

namespace state {

namespace internal {

/**
 * @brief Struct Kinematics
 *
 * Kinematics state of a multirotor vehicle are the state variables that
 * describe the position and orientation of the vehicle, as well as their
 * derivatives.
 *
 * @tparam T Precision
 */
template <typename T = double>
struct Kinematics {
  // Check if T is a numeric type (floating-point or integral)
  static_assert(std::is_floating_point<T>::value,
                "MotorParams must be used with a floating-point type");

  Eigen::Matrix<T, 3, 1> position         = Eigen::Matrix<T, 3, 1>::Zero();      // m
  Eigen::Quaternion<T> orientation        = Eigen::Quaternion<T>::Identity();    // quaternion (rad)
  Eigen::Matrix<T, 3, 1> linear_velocity  = Eigen::Matrix<T, 3, 1>::Zero();      // m/s
  Eigen::Matrix<T, 3, 1> angular_velocity = Eigen::Matrix<T, 3, 1>::Zero();      // rad/s
  Eigen::Matrix<T, 3, 1> linear_acceleration  = Eigen::Matrix<T, 3, 1>::Zero();  // m/s^2
  Eigen::Matrix<T, 3, 1> angular_acceleration = Eigen::Matrix<T, 3, 1>::Zero();  // rad/s^2
};  // struct Kinematics

/**
 * @brief Struct Dynamics
 *
 * Dynamics state of a multirotor vehicle are the state variables that describe
 * the forces and torques applied to the vehicle.
 *
 * @tparam T Precision
 */
template <typename T = double>
struct Dynamics {
  // Check if T is a numeric type (floating-point or integral)
  static_assert(std::is_floating_point<T>::value,
                "MotorParams must be used with a floating-point type");

  Eigen::Matrix<T, 3, 1> force  = Eigen::Matrix<T, 3, 1>::Zero();  // N
  Eigen::Matrix<T, 3, 1> torque = Eigen::Matrix<T, 3, 1>::Zero();  // N · m
};                                                                 // struct Dynamics

/**
 * @brief Struct Actuators
 *
 * Actuators state of a multirotor vehicle are the state variables that describe
 * the state of the actuators of the vehicle.
 *
 * @tparam T Precision
 * @tparam num_rotors Number of rotors of the multirotor vehicle
 */
template <typename T = double, int num_rotors = 4>
struct Actuators {
  // Check if T is a numeric type (floating-point or integral)
  static_assert(std::is_floating_point<T>::value,
                "MotorParams must be used with a floating-point type");

  Eigen::Matrix<T, num_rotors, 1> motor_angular_velocity =
      Eigen::Matrix<T, num_rotors, 1>::Zero();  // rad/s
  Eigen::Matrix<T, num_rotors, 1> motor_angular_acceleration =
      Eigen::Matrix<T, num_rotors, 1>::Zero();  // rad/s^2
};                                              // struct Actuation

template <typename T = double>
void quaternion_to_Euler(const Eigen::Quaternion<T>& _quaternion, T& roll, T& pitch, T& yaw) {
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

template <typename T = double>
std::ostream& operator<<(std::ostream& os, const Kinematics<T>& kinematics) {
  Eigen::Matrix<T, 3, 1> euler_angles;
  quaternion_to_Euler(kinematics.orientation, euler_angles.x(), euler_angles.y(), euler_angles.z());
  os << kinematics.position.x() << "," << kinematics.position.y() << "," << kinematics.position.z()
     << "," << euler_angles.x() << "," << euler_angles.y() << "," << euler_angles.z() << ","
     << kinematics.linear_velocity.x() << "," << kinematics.linear_velocity.y() << ","
     << kinematics.linear_velocity.z() << "," << kinematics.angular_velocity.x() << ","
     << kinematics.angular_velocity.y() << "," << kinematics.angular_velocity.z() << ","
     << kinematics.linear_acceleration.x() << "," << kinematics.linear_acceleration.y() << ","
     << kinematics.linear_acceleration.z() << "," << kinematics.angular_acceleration.x() << ","
     << kinematics.angular_acceleration.y() << "," << kinematics.angular_acceleration.z();
  return os;
}

template <typename T = double>
std::ostream& print_kinematics(std::ostream& os, const Kinematics<T>& kinematics) {
  Eigen::Matrix<T, 3, 1> euler_angles;
  quaternion_to_Euler(kinematics.orientation, euler_angles.x(), euler_angles.y(), euler_angles.z());
  os << "position_x: " << kinematics.position.x() << ", "
     << "position_y: " << kinematics.position.y() << ", "
     << "position_z: " << kinematics.position.z() << ", "
     << "orientation_roll: " << euler_angles.x() << ", "
     << "orientation_pitch: " << euler_angles.y() << ", "
     << "orientation_yaw: " << euler_angles.z() << ", "
     << "linear_velocity_x: " << kinematics.linear_velocity.x() << ", "
     << "linear_velocity_y: " << kinematics.linear_velocity.y() << ", "
     << "linear_velocity_z: " << kinematics.linear_velocity.z() << ", "
     << "angular_velocity_x: " << kinematics.angular_velocity.x() << ", "
     << "angular_velocity_y: " << kinematics.angular_velocity.y() << ", "
     << "angular_velocity_z: " << kinematics.angular_velocity.z() << ", "
     << "linear_acceleration_x: " << kinematics.linear_acceleration.x() << ", "
     << "linear_acceleration_y: " << kinematics.linear_acceleration.y() << ", "
     << "linear_acceleration_z: " << kinematics.linear_acceleration.z() << ", "
     << "angular_acceleration_x: " << kinematics.angular_acceleration.x() << ", "
     << "angular_acceleration_y: " << kinematics.angular_acceleration.y() << ", "
     << "angular_acceleration_z: " << kinematics.angular_acceleration.z();
  return os;
}

template <typename T = double>
std::ostream& operator<<(std::ostream& os, const Dynamics<T>& dynamics) {
  os << dynamics.force.x() << "," << dynamics.force.y() << "," << dynamics.force.z() << ","
     << dynamics.torque.x() << "," << dynamics.torque.y() << "," << dynamics.torque.z();
  return os;
}

template <typename T = double>
std::ostream& print_dynamics(std::ostream& os, const Dynamics<T>& dynamics) {
  os << "force_x: " << dynamics.force.x() << ", "
     << "force_y: " << dynamics.force.y() << ", "
     << "force_z: " << dynamics.force.z() << ", "
     << "torque_x: " << dynamics.torque.x() << ", "
     << "torque_y: " << dynamics.torque.y() << ", "
     << "torque_z: " << dynamics.torque.z();
  return os;
}

template <typename T = double, int num_rotors = 4>
std::ostream& operator<<(std::ostream& os, const Actuators<T, num_rotors>& actuators) {
  for (int i = 0; i < num_rotors; i++) {
    os << actuators.motor_angular_velocity[i] << ",";
  }
  for (int i = 0; i < num_rotors; i++) {
    os << actuators.motor_angular_acceleration[i];
    if (i < num_rotors - 1) {
      os << ",";
    }
  }
  return os;
}

template <typename T = double, int num_rotors = 4>
std::ostream& print_actuators(std::ostream& os, const Actuators<T, num_rotors>& actuators) {
  for (int i = 0; i < num_rotors; i++) {
    os << "motor_" << i << "_angular_velocity: " << actuators.motor_angular_velocity[i];
    if (i < num_rotors - 1) {
      os << ", ";
    }
  }
  for (int i = 0; i < num_rotors; i++) {
    os << "motor_" << i << "_angular_acceleration: " << actuators.motor_angular_acceleration[i];
    if (i < num_rotors - 1) {
      os << ", ";
    }
  }
  return os;
}

}  // namespace internal

/**
 * @brief Struct State
 *
 * Multirotor state is the state of a multirotor vehicle, which is composed of
 * the kinematics, dynamics and actuators state.
 *
 * @tparam T Precision
 * @tparam num_rotors Number of rotors of the multirotor vehicle
 */
template <typename T = double, int num_rotors = 4>
struct State {
  // Check if T is a numeric type (floating-point or integral)
  static_assert(std::is_floating_point<T>::value,
                "MotorParams must be used with a floating-point type");

  internal::Kinematics<T> kinematics;
  internal::Dynamics<T> dynamics;
  internal::Actuators<T, num_rotors> actuators;
};  // struct state

template <typename T = double, int num_rotors = 4>
std::ostream& operator<<(std::ostream& os, const State<T, num_rotors>& state) {
  os << state.kinematics << "," << state.dynamics << "," << state.actuators;
  return os;
}

template <typename T = double, int num_rotors = 4>
void print_state(const State<T, num_rotors>& state) {
  state::internal::print_kinematics(std::cout, state.kinematics);
  state::internal::print_dynamics(std::cout, state.dynamics);
  state::internal::print_actuators(std::cout, state.actuators);
}

}  // namespace state

}  // namespace multirotor

#endif  // MULTIROTOR_DYNAMIC_MODEL_COMMON_STATE_HPP_
