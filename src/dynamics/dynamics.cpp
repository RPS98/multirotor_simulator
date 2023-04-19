/*!*******************************************************************************************
 *  \file       dynamics.cpp
 *  \brief      Quadrotor dynamics implementation file.
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

#include "dynamics/dynamics.hpp"

using namespace quadrotor;

Dynamics::Dynamics(std::shared_ptr<Model> model, std::shared_ptr<State> state) {
  model_ = model;
  state_ = state;
}

Dynamics::~Dynamics() {
  // TODO
}

void Dynamics::process_euler_explicit(const actuation::MotorW &actuation, const float dt) {
  Eigen::Vector4f motor_angular_velocity_desired = actuation.angular_velocity;
  *last_state_                                   = *state_;

  // Initialize stochastic noise
  model_->set_stochastic_noise(dt);

  // Process actuation
  utils::clamp_vector(motor_angular_velocity_desired, model_->motor_min_speed,
                      model_->motor_max_speed);

  // Compute system derivatives
  Eigen::Vector4f motor_angular_velocity_derivative = get_motors_angular_velocity_derivative(
      motor_angular_velocity_desired, state_->actuators.motor_angular_velocity,
      model_->motor_time_constant);

  Eigen::Vector3f angular_velocity_derivative = get_vehicle_angular_velocity_derivative(
      model_->vehicle_inertia, state_->kinematics.angular_velocity,
      model_->get_motor_torque(state_->actuators.motor_angular_velocity,
                               motor_angular_velocity_derivative),
      model_->get_aerodynamic_moment(state_->kinematics.angular_velocity),
      model_->get_stochastic_force(), state_->dynamics.torque);

  Eigen::Vector3f linear_velocity_derivative = get_vehicle_linear_velocity_derivative(
      model_->mass, state_->kinematics.orientation, model_->motor_thrust_coefficient,
      state_->actuators.motor_angular_velocity, model_->vehicle_drag_coefficient,
      state_->kinematics.linear_velocity, model_->get_gravity_force(),
      model_->get_stochastic_force(), state_->dynamics.force);

  Eigen::Vector4f orientation_derivative = get_vehicle_orientation_derivative(
      state_->kinematics.orientation, state_->kinematics.angular_velocity);

  Eigen::Vector3f position_derivative =
      get_vehicle_position_derivative(state_->kinematics.linear_velocity);

  // Integrate system
  state_->actuators.motor_angular_velocity += motor_angular_velocity_derivative * dt;
  state_->kinematics.angular_velocity += angular_velocity_derivative * dt;
  state_->kinematics.linear_velocity += linear_velocity_derivative * dt;
  state_->kinematics.orientation.coeffs() += orientation_derivative * dt;
  state_->kinematics.orientation.normalize();
  state_->kinematics.position += position_derivative * dt;

  state_->kinematics.angular_acceleration = angular_velocity_derivative;
  state_->kinematics.linear_acceleration  = linear_velocity_derivative;
}

/**
 * @brief Compute the motor angular velocity derivative as a first order system
 *
 * @param reference Vector of desired angular velocities
 * @param stat Vector of current angular velocities
 * @param time_constant Time constant of the system
 *
 * @return Eigen::Vector4f Motor angular velocity derivative
 */
inline Eigen::Vector4f Dynamics::get_motors_angular_velocity_derivative(
    const Eigen::Vector4f &desired_angular_velocity,
    const Eigen::Vector4f &current_angular_velocity,
    const float time_constant) {
  return (desired_angular_velocity - current_angular_velocity) / time_constant;
}

/**
 * @brief Compute the vehicle angular velocity derivative in body frame
 *
 * @param vehicle_inertia_matrix Inertia matrix of the vehicle
 * @param vehicle_angular_velocity Vehicle angular velocity in body frame
 * @param vehicle_control_moment Vehicle control moment in body frame
 * @param vehicle_aerodynamic_moment Vehicle aerodynamic moment in body frame
 * @param vehicle_stochastic_force Vehicle stochastic force in body frame
 * @param vehicle_total_torque Output vehicle total torque in body frame
 *
 * @return Eigen::Vector3f Vehicle angular velocity derivative
 */
Eigen::Vector3f Dynamics::get_vehicle_angular_velocity_derivative(
    const Eigen::Matrix3f &vehicle_inertia_matrix,
    const Eigen::Vector3f &vehicle_angular_velocity,
    const Eigen::Vector3f &vehicle_control_moment,
    const Eigen::Vector3f &vehicle_aerodynamic_moment,
    const Eigen::Vector3f &vehicle_stochastic_force,
    Eigen::Vector3f &vehicle_total_torque) {
  Eigen::Vector3f angular_velocity_derivative;

  vehicle_total_torque =
      vehicle_control_moment + vehicle_aerodynamic_moment + vehicle_stochastic_force -
      vehicle_angular_velocity.cross(vehicle_inertia_matrix * vehicle_angular_velocity);

  angular_velocity_derivative = vehicle_inertia_matrix.inverse() * vehicle_total_torque;

  return angular_velocity_derivative;
}

/**
 * @brief Compute the vehicle position derivative
 *
 * @param linear_velocity Vehicle linear velocity in earth frame
 *
 * @return Eigen::Vector3f Vehicle position derivative
 */
inline Eigen::Vector3f Dynamics::get_vehicle_position_derivative(
    const Eigen::Vector3f &linear_velocity) {
  return linear_velocity;
}

/**
 * @brief Compute the vehicle orientation derivative
 *
 * @param orientation Vehicle orientation
 * @param angular_velocity Vehicle angular velocity in body frame
 *
 * @return Eigen::Vector4f Vehicle orientation derivative
 */
inline Eigen::Vector4f Dynamics::get_vehicle_orientation_derivative(
    const Eigen::Quaternionf &orientation,
    const Eigen::Vector3f &angular_velocity) {
  return utils::get_quaternion_derivative(orientation, angular_velocity);
}

/**
 * @brief Compute the vehicle linear velocity derivative
 *
 * @param mass Mass of the vehicle in kg
 * @param orientation Vehicle orientation in earth frame
 * @param motor_thrust_coefficient Motor thrust coefficient
 * @param motor_angular_velocity Motor angular velocity
 * @param vehicle_drag_coefficient Vehicle drag coefficient
 * @param vehicle_linear_velocity Vehicle linear velocity in earth frame
 * @param gravity_force Gravity force in earth frame
 * @param stochastic_force Stochastic force in earth frame
 *
 * @return Eigen::Vector3f Vehicle linear velocity derivative
 */
Eigen::Vector3f Dynamics::get_vehicle_linear_velocity_derivative(
    const float mass,
    const Eigen::Quaternionf &orientation,
    const float motor_thrust_coefficient,
    const Eigen::Vector4f &motor_angular_velocity,
    const float vehicle_drag_coefficient,
    const Eigen::Vector3f &vehicle_linear_velocity,
    const Eigen::Vector3f &gravity_force,
    const Eigen::Vector3f &stochastic_force,
    Eigen::Vector3f &vehicle_total_force) {
  // Compute the thrust force in earth frame
  Eigen::Vector3f thrust_force =
      orientation * Model::get_thrust_force(motor_thrust_coefficient, motor_angular_velocity);

  // Compute the drag force
  Eigen::Vector3f drag_force =
      Model::get_drag_force(vehicle_drag_coefficient, vehicle_linear_velocity);

  // Compute the total force
  vehicle_total_force = thrust_force + gravity_force + drag_force + stochastic_force;

  // Compute the linear velocity derivative
  return vehicle_total_force / mass;
}
