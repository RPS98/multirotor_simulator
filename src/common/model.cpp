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

#include "common/model.hpp"

using namespace quadrotor;

/**
 * @brief Construct a quadrotor model
 *
 * @param thrust_coefficient Motor thrust coefficient in N / (rad/s)^2
 * @param torque_coefficient Motor torque coefficient in N · m / (rad/s)^2
 * @param min_motor_speed Minimum motor speed in rad/s
 * @param max_motor_speed Maximum motor speed in rad/s
 * @param motor_time_constant Motor time constant in s
 * @param motor_rotational_inertia Motor rotational inertia in kg · m^2
 * @param vehicle_mass Vehicle mass in kg
 * @param vehicle_inertia Vehicle inertia in kg · m^2
 * @param aero_moment_coefficient Aerodynamic moment coefficient in N · m / (rad/s)^2
 * @param drag_coefficient Aerodynamic drag coefficient in N / (rad/s)^2
 * @param moment_process_noise_auto_correlation Process noise auto-correlation for moments
 * @param force_process_noise_auto_correlation Process noise auto-correlation for forces
 * @param gravity Gravity vector in m/s^2
 */
Model::Model(float thrust_coefficient,
             float torque_coefficient,
             float min_motor_speed,
             float max_motor_speed,
             float motor_time_constant,
             float motor_rotational_inertia,
             float motor_dx,
             float motor_dy,
             float vehicle_mass,
             const Eigen::Matrix3f& vehicle_inertia,
             float aero_moment_coefficient,
             float drag_coefficient,
             float moment_process_noise_auto_correlation,
             float force_process_noise_auto_correlation,
             const Eigen::Vector3f& gravity)
    : vehicle_mass(vehicle_mass), vehicle_inertia(vehicle_inertia),
      aero_moment_coefficient(aero_moment_coefficient),
      vehicle_drag_coefficient(vehicle_drag_coefficient),
      moment_process_noise_auto_correlation(moment_process_noise_auto_correlation),
      force_process_noise_auto_correlation(force_process_noise_auto_correlation), gravity(gravity) {
  motors_frame_thrust_coefficient_matrix << 0.0f, motor_dy * thrust_coefficient, 0.0f,
      -motor_dy * thrust_coefficient, -motor_dx * thrust_coefficient, 0.0f,
      motor_dx * thrust_coefficient, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f;

  motors_frame_torque_coefficient_matrix << 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
      torque_coefficient, -torque_coefficient, torque_coefficient, -torque_coefficient;

  motors_frame_inertia_matrix << 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
      motor_rotational_inertia, -motor_rotational_inertia, motor_rotational_inertia,
      -motor_rotational_inertia;
}

Model::~Model() {
  // TODO
}

/**
 * @brief Generate random force and moment noise
 *
 * @param dt Time step in s
 */
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

/**
 * @brief Get the stochastic force in the body frame
 *
 * @return Eigen::Vector3f
 */
inline Eigen::Vector3f Model::get_stochastic_force() const { return stoch_force; }

/**
 * @brief Get the stochastic moment in the body frame
 *
 * @return Eigen::Vector3f
 */
inline Eigen::Vector3f Model::get_stochastic_moment() const { return stoch_moment; }

/**
 * @brief Get the gravity force in the earth frame
 *
 * @return Eigen::Vector3f Gravity force in N in the earth frame
 */
inline Eigen::Vector3f Model::get_gravity_force() const { return gravity; }

/**
 * @brief Get the thrust force in the body frame produced by the motors
 *
 * @param motors_angular_velocity Motors angular velocity in rad/s
 *
 * @return Eigen::Vector3f Thrust force in the body frame
 */
inline Eigen::Vector3f Model::get_thrust_force(
    const Eigen::Vector4f& motors_angular_velocity) const {
  return get_thrust_force(motor_thrust_coefficient, motors_angular_velocity);
}

/**
 * @brief Get the thrust force in the body frame produced by the motors
 *
 * @param motors_thrust_coefficient Motors thrust coefficient
 * @param motors_angular_velocity Motors angular velocity
 *
 * @return Eigen::Vector3f Thrust force in the body frame
 */
Eigen::Vector3f Model::get_thrust_force(const float motors_thrust_coefficient,
                                        const Eigen::Vector4f& motors_angular_velocity) {
  return Eigen::Vector3f(0.0f, 0.0f, motors_thrust_coefficient * motors_angular_velocity.sum());
}

/**
 * @brief Get the drag force in the velocity frame produced by the environment
 *
 * @param vehicle_linear_velocity Vehicle linear velocity
 *
 * @return Eigen::Vector3f Drag force in the velocity frame
 */
inline Eigen::Vector3f Model::get_drag_force(const Eigen::Vector3f& vehicle_linear_velocity) const {
  return get_drag_force(vehicle_drag_coefficient, vehicle_linear_velocity);
}

/**
 * @brief Get the drag force in the velocity frame produced by the environment
 *
 * @param vehicle_drag_coefficient Vehicle drag coefficient
 * @param vehicle_linear_velocity Vehicle linear velocity
 *
 * @return Eigen::Vector3f Drag force in the velocity frame
 */
Eigen::Vector3f Model::get_drag_force(const float _vehicle_drag_coefficient,
                                      const Eigen::Vector3f& _vehicle_linear_velocity) {
  return -_vehicle_drag_coefficient * _vehicle_linear_velocity.norm() * _vehicle_linear_velocity;
}

/**
 * @brief Get the aerodynamic moment in the body frame produced by the environment
 *
 * @param vehicle_angular_velocity Vehicle angular velocity in the body frame
 *
 * @return Eigen::Vector3f Aerodynamic moment in the body frame
 */
inline Eigen::Vector3f Model::get_aerodynamic_moment(
    const Eigen::Vector3f& vehicle_angular_velocity) const {
  return get_aerodynamic_moment(aero_moment_coefficient, vehicle_angular_velocity);
}

/**
 * @brief Get aerodynamic moment in the body frame produced by the environment
 *
 * @param vehicle_aerodynamic_coefficient Vehicle aerodynamic coefficient
 * @param vehicle_angular_velocity Vehicle angular velocity in the body frame
 *
 * @return Eigen::Vector3f Aerodynamic moment in the body frame
 */
Eigen::Vector3f Model::get_aerodynamic_moment(const float vehicle_aerodynamic_coefficient,
                                              const Eigen::Vector3f& vehicle_angular_velocity) {
  return -vehicle_aerodynamic_coefficient * vehicle_angular_velocity.norm() *
         vehicle_angular_velocity;
}

/**
 * @brief Get the motor torque in the body frame produced by the motors
 * 
 * @param motors_angular_velocity Angular velocity of the motors in rad/s
 * @param motors_angular_acceleration Angular acceleration of the motors in rad/s^2
 * 
 * @return Eigen::Vector3f Motor torque in the body frame
 */
Eigen::Vector3f Model::get_motor_torque(const Eigen::Vector4f& motors_angular_velocity,
                                        const Eigen::Vector4f& motors_angular_acceleration) const {
  Eigen::Vector4f motors_angular_velocity_squared = motors_angular_velocity.array().square();

  return (motors_frame_thrust_coefficient_matrix + motors_frame_torque_coefficient_matrix) *
             motors_angular_velocity_squared +
         motors_frame_inertia_matrix * motors_angular_acceleration;
}