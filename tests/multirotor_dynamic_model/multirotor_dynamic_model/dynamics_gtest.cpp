/*!*******************************************************************************************
 *  \file       dynamics_gtest.cpp
 *  \brief      Dynamics unit test
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

#include <gtest/gtest.h>
#include <memory>
#include "multirotor_dynamic_model/dynamics.hpp"

template <typename P = double, int num_rotors = 4>
using State = multirotor::state::State<P, num_rotors>;
template <typename P = double>
using MotorParams = multirotor::model::MotorParams<P>;
template <typename P = double, int num_rotors = 4>
using ModelParams = multirotor::model::ModelParams<P, num_rotors>;
template <typename P = double, int num_rotors = 4>
using Model = multirotor::model::Model<P, num_rotors>;
template <typename P = double, int num_rotors = 4>
using DynamicsParams = multirotor::dynamics::DynamicsParams<P, num_rotors>;
template <typename P = double, int num_rotors = 4>
using Dynamics = multirotor::dynamics::Dynamics<P, num_rotors>;

// Class child of Dynamics to access protected methods and convert to public
template <typename T = double, int num_rotors = 4>
class PublicDynamics : public Dynamics<T, num_rotors> {
public:
  // Inherit the constructor from the base class
  using Dynamics<T, num_rotors>::Dynamics;

  // Make the protected functions public in the child class
  using Dynamics<T, num_rotors>::clamp_motors_angular_velocity;
  using Dynamics<T, num_rotors>::get_motors_angular_velocity_derivative;
  using Dynamics<T, num_rotors>::get_vehicle_angular_velocity_derivative;
  using Dynamics<T, num_rotors>::get_thrust_force_in_earth_frame;
  using Dynamics<T, num_rotors>::get_vehicle_linear_velocity_derivative;
  using Dynamics<T, num_rotors>::get_vehicle_orientation_derivative;
  using Dynamics<T, num_rotors>::get_vehicle_position_derivative;
};

TEST(Dynamics, constructor_default) { EXPECT_NO_THROW(Dynamics<> dynamics = Dynamics<>()); }

TEST(Dynamics, constructor_default_template) {
  ModelParams<> model_params;
  Model<> model_ = Model<>(model_params);
  State<> state_ = State<>();
  EXPECT_NO_THROW(Dynamics<> dynamics = Dynamics<>(model_, state_));
}

TEST(Dynamics, constructor_template) {
  ModelParams<double, 4> model_params;
  Model<double, 4> model_      = Model<double, 4>(model_params);
  State<double, 4> state_      = State<double, 4>();
  Dynamics<double, 4> dynamics = Dynamics<double, 4>(model_, state_);
}

TEST(Dynamics, constructor_params) {
  DynamicsParams<> dynamics_params;
  EXPECT_NO_THROW(Dynamics<> dynamics = Dynamics<>(dynamics_params));
}

Model<double, 4> get_model() {
  const double thrust_coefficient = 1.91e-6;
  const double torque_coefficient = 2.6e-7;
  const double x_dist             = 0.08;
  const double y_dist             = 0.08;
  const double min_speed          = 0.0;
  const double max_speed          = 2200.0;
  const double time_constant      = 0.02;
  const double rotational_inertia = 6.62e-6;

  std::vector<MotorParams<double>> motors = Model<double, 4>::create_quadrotor_x_config(
      thrust_coefficient, torque_coefficient, x_dist, y_dist, min_speed, max_speed, time_constant,
      rotational_inertia);

  ModelParams<double, 4> model_params;
  // Motor characteristics
  model_params.motors_params = motors;
  // Vehicle characteristics
  model_params.vehicle_mass    = 1.0;
  model_params.vehicle_inertia = Eigen::Vector3d(0.0049, 0.0049, 0.0069).asDiagonal();
  // Aerodynamic characteristics
  model_params.vehicle_aero_moment_coefficient = Eigen::Vector3d(0.003, 0.003, 0.003).asDiagonal();
  model_params.vehicle_drag_coefficient        = 0.1;
  // Environment characteristics
  model_params.gravity                               = Eigen::Vector3d(0.0, 0.0, -9.81);
  model_params.force_process_noise_auto_correlation  = 5.00e-5;
  model_params.moment_process_noise_auto_correlation = 1.25e-7;

  // Create the model
  Model<double, 4> model = Model<double, 4>(model_params);
  return model;
}

TEST(Dynamics, public_methods) {
  Model<double, 4> model_ = get_model();
  State<double, 4> state_ = State<double, 4>();

  Dynamics<double, 4> dynamics(model_, state_);
  Eigen::Vector4d actuation_motors_angular_velocity = Eigen::Vector4d::Ones();

  // Test publics methods
  EXPECT_ANY_THROW(dynamics.process_euler_explicit(actuation_motors_angular_velocity, 0.0,
                                                   Eigen::Vector3d::Zero()));
  EXPECT_NO_THROW(dynamics.process_euler_explicit(actuation_motors_angular_velocity, 0.001,
                                                  Eigen::Vector3d::Zero()));

  // Getters
  EXPECT_NO_THROW(dynamics.get_state());
  EXPECT_NO_THROW(dynamics.get_model());
  EXPECT_NO_THROW(dynamics.get_model_const());

  // Setters
  EXPECT_NO_THROW(dynamics.set_state(state_));
  EXPECT_NO_THROW(dynamics.set_model(model_));
}

TEST(Dynamics, protected_methods) {
  Model<double, 4> model_ = get_model();
  State<double, 4> state_ = State<double, 4>();

  PublicDynamics<double, 4> dynamics(model_, state_);

  Eigen::Vector4d actuation_motors_angular_velocity = Eigen::Vector4d::Ones();
  Eigen::Vector3d external_force                    = Eigen::Vector3d::Zero();

  // Test protected methods
  EXPECT_NO_THROW(dynamics.clamp_motors_angular_velocity(actuation_motors_angular_velocity,
                                                         model_.get_motors()));
  EXPECT_NO_THROW(dynamics.get_motors_angular_velocity_derivative(
      actuation_motors_angular_velocity, state_.actuators.motor_angular_velocity,
      model_.get_motors()));
  EXPECT_NO_THROW(dynamics.get_vehicle_angular_velocity_derivative(
      state_.kinematics.angular_velocity, model_.get_vehicle_inertia(),
      model_.get_torque_by_motors(state_.actuators.motor_angular_velocity,
                                  state_.actuators.motor_angular_acceleration),
      model_.get_aerodynamic_moment(state_.kinematics.angular_velocity),
      model_.get_stochastic_force(), state_.dynamics.torque));
  EXPECT_NO_THROW(
      dynamics.get_thrust_force_in_earth_frame(state_.actuators.motor_angular_velocity));
  EXPECT_NO_THROW(dynamics.get_vehicle_linear_velocity_derivative(
      model_.get_mass(),
      dynamics.get_thrust_force_in_earth_frame(state_.actuators.motor_angular_velocity),
      model_.get_drag_force(state_.kinematics.linear_velocity), model_.get_gravity(),
      model_.get_stochastic_force(), external_force, state_.dynamics.force));
  EXPECT_NO_THROW(dynamics.get_vehicle_orientation_derivative(state_.kinematics.orientation,
                                                              state_.kinematics.angular_velocity));
  EXPECT_NO_THROW(dynamics.get_vehicle_position_derivative(state_.kinematics.linear_velocity));
}

TEST(Dynamics, protected_methods_clamp_motors_angular_velocity) {
  Model<double, 4> model_ = get_model();
  State<double, 4> state_ = State<double, 4>();

  const double min_speed              = 0.0;
  const double max_speed              = 2500.0;
  ModelParams<double, 4> model_params = model_.get_params();
  for (auto &motor : model_params.motors_params) {
    motor.min_speed = min_speed;
    motor.max_speed = max_speed;
  }
  model_.update_params(model_params);

  // Check if min and max speed has been set
  for (auto &motor : model_.get_motors()) {
    EXPECT_EQ(motor.min_speed, min_speed);
    EXPECT_EQ(motor.max_speed, max_speed);
  }

  PublicDynamics<double, 4> dynamics(model_, state_);

  Eigen::Vector4d actuation_motors_angular_velocity = Eigen::Vector4d::Ones();
  Eigen::Vector4d motor_angular_velocity_desired    = Eigen::Vector4d::Ones();

  // Test clamp_motors_angular_velocity method
  EXPECT_NO_THROW(dynamics.clamp_motors_angular_velocity(actuation_motors_angular_velocity,
                                                         model_.get_motors()));

  // Test not clamping
  actuation_motors_angular_velocity = Eigen::Vector4d::Ones() * 1000.0;
  motor_angular_velocity_desired    = dynamics.clamp_motors_angular_velocity(
         actuation_motors_angular_velocity, model_.get_motors());

  EXPECT_EQ(motor_angular_velocity_desired(0), actuation_motors_angular_velocity(0));
  EXPECT_EQ(motor_angular_velocity_desired(1), actuation_motors_angular_velocity(1));
  EXPECT_EQ(motor_angular_velocity_desired(2), actuation_motors_angular_velocity(2));
  EXPECT_EQ(motor_angular_velocity_desired(3), actuation_motors_angular_velocity(3));

  // Test clamping upper
  actuation_motors_angular_velocity = Eigen::Vector4d::Ones() * 10000.0;
  motor_angular_velocity_desired    = dynamics.clamp_motors_angular_velocity(
         actuation_motors_angular_velocity, model_.get_motors());

  EXPECT_EQ(motor_angular_velocity_desired(0), max_speed);
  EXPECT_EQ(motor_angular_velocity_desired(1), max_speed);
  EXPECT_EQ(motor_angular_velocity_desired(2), max_speed);
  EXPECT_EQ(motor_angular_velocity_desired(3), max_speed);

  // Test clamping lower
  actuation_motors_angular_velocity = Eigen::Vector4d::Ones() * -10000.0;
  motor_angular_velocity_desired    = dynamics.clamp_motors_angular_velocity(
         actuation_motors_angular_velocity, model_.get_motors());

  EXPECT_EQ(motor_angular_velocity_desired(0), min_speed);
  EXPECT_EQ(motor_angular_velocity_desired(1), min_speed);
  EXPECT_EQ(motor_angular_velocity_desired(2), min_speed);
  EXPECT_EQ(motor_angular_velocity_desired(3), min_speed);

  // Test not clamping, clamping upper and clamping lower
  actuation_motors_angular_velocity    = Eigen::Vector4d::Ones() * 1000.0;
  actuation_motors_angular_velocity(0) = 10000.0;
  actuation_motors_angular_velocity(1) = -10000.0;
  motor_angular_velocity_desired       = dynamics.clamp_motors_angular_velocity(
            actuation_motors_angular_velocity, model_.get_motors());

  EXPECT_EQ(motor_angular_velocity_desired(0), max_speed);
  EXPECT_EQ(motor_angular_velocity_desired(1), min_speed);
  EXPECT_EQ(motor_angular_velocity_desired(2), actuation_motors_angular_velocity(2));
  EXPECT_EQ(motor_angular_velocity_desired(3), actuation_motors_angular_velocity(3));
}

TEST(Dynamics, protected_methods_get_motors_angular_velocity_derivative) {
  Model<double, 4> model_ = get_model();
  State<double, 4> state_ = State<double, 4>();

  // Initialize actuators state
  state_.actuators.motor_angular_velocity = Eigen::Vector4d::Ones() * 1000.0;

  PublicDynamics<double, 4> dynamics(model_, state_);

  Eigen::Vector4d motor_angular_velocity_desired    = Eigen::Vector4d::Ones();
  Eigen::Vector4d motor_angular_velocity_derivative = Eigen::Vector4d::Ones();

  // Test get_motors_angular_velocity_derivative method
  EXPECT_NO_THROW(dynamics.get_motors_angular_velocity_derivative(
      motor_angular_velocity_desired, state_.actuators.motor_angular_velocity,
      model_.get_motors()));

  // Test accelerate motors
  motor_angular_velocity_desired    = state_.actuators.motor_angular_velocity * 2.0;
  motor_angular_velocity_derivative = dynamics.get_motors_angular_velocity_derivative(
      motor_angular_velocity_desired, state_.actuators.motor_angular_velocity, model_.get_motors());

  EXPECT_GT(motor_angular_velocity_derivative(0), 0.0);
  EXPECT_GT(motor_angular_velocity_derivative(1), 0.0);
  EXPECT_GT(motor_angular_velocity_derivative(2), 0.0);
  EXPECT_GT(motor_angular_velocity_derivative(3), 0.0);

  // Test decelerate motors
  motor_angular_velocity_desired    = state_.actuators.motor_angular_velocity * 0.5;
  motor_angular_velocity_derivative = dynamics.get_motors_angular_velocity_derivative(
      motor_angular_velocity_desired, state_.actuators.motor_angular_velocity, model_.get_motors());

  EXPECT_LT(motor_angular_velocity_derivative(0), 0.0);
  EXPECT_LT(motor_angular_velocity_derivative(1), 0.0);
  EXPECT_LT(motor_angular_velocity_derivative(2), 0.0);
  EXPECT_LT(motor_angular_velocity_derivative(3), 0.0);

  // Test not accelerate motors
  motor_angular_velocity_desired    = state_.actuators.motor_angular_velocity;
  motor_angular_velocity_derivative = dynamics.get_motors_angular_velocity_derivative(
      motor_angular_velocity_desired, state_.actuators.motor_angular_velocity, model_.get_motors());

  EXPECT_EQ(motor_angular_velocity_derivative(0), 0.0);
  EXPECT_EQ(motor_angular_velocity_derivative(1), 0.0);
  EXPECT_EQ(motor_angular_velocity_derivative(2), 0.0);
  EXPECT_EQ(motor_angular_velocity_derivative(3), 0.0);

  // Test accelerate two motors and decelerate the other two
  motor_angular_velocity_desired(0) = state_.actuators.motor_angular_velocity(0) * 2.0;
  motor_angular_velocity_desired(1) = state_.actuators.motor_angular_velocity(1) * 2.0;
  motor_angular_velocity_desired(2) = state_.actuators.motor_angular_velocity(2) * 0.5;
  motor_angular_velocity_desired(3) = state_.actuators.motor_angular_velocity(3) * 0.5;

  motor_angular_velocity_derivative = dynamics.get_motors_angular_velocity_derivative(
      motor_angular_velocity_desired, state_.actuators.motor_angular_velocity, model_.get_motors());

  EXPECT_GT(motor_angular_velocity_derivative(0), 0.0);
  EXPECT_GT(motor_angular_velocity_derivative(1), 0.0);
  EXPECT_LT(motor_angular_velocity_derivative(2), 0.0);
  EXPECT_LT(motor_angular_velocity_derivative(3), 0.0);
}

TEST(Dynamics, protected_methods_get_vehicle_angular_velocity_derivative) {
  Model<double, 4> model_ = get_model();
  State<double, 4> state_ = State<double, 4>();

  PublicDynamics<double, 4> dynamics(model_, state_);
  Eigen::Vector3d angular_velocity_derivative = Eigen::Vector3d::Ones();

  const double motor_speed_current = 1000.0;
  const double motor_accel         = 100.0;
  const double max_speed           = model_.get_motors()[0].max_speed;
  const double min_speed           = model_.get_motors()[0].min_speed;

  // Initialize actuators state
  state_.actuators.motor_angular_velocity     = Eigen::Vector4d::Ones() * motor_speed_current;
  state_.actuators.motor_angular_acceleration = Eigen::Vector4d::Zero();

  // Test get_motors_angular_velocity_derivative method
  EXPECT_NO_THROW(dynamics.get_vehicle_angular_velocity_derivative(
      state_.kinematics.angular_velocity, model_.get_vehicle_inertia(),
      model_.get_torque_by_motors(state_.actuators.motor_angular_velocity,
                                  state_.actuators.motor_angular_acceleration),
      model_.get_aerodynamic_moment(state_.kinematics.angular_velocity),
      model_.get_stochastic_force(), state_.dynamics.torque));

  // Vehicle has a X-Configuration
  // Test decelerate vehicle in x axis:
  // - Accelerate motors 0 and 3
  // - Decelerate motors 1 and 2
  state_.actuators.motor_angular_velocity(0)     = max_speed;
  state_.actuators.motor_angular_velocity(1)     = min_speed;
  state_.actuators.motor_angular_velocity(2)     = min_speed;
  state_.actuators.motor_angular_velocity(3)     = max_speed;
  state_.actuators.motor_angular_acceleration(0) = motor_accel;
  state_.actuators.motor_angular_acceleration(1) = -motor_accel;
  state_.actuators.motor_angular_acceleration(2) = -motor_accel;
  state_.actuators.motor_angular_acceleration(3) = motor_accel;

  angular_velocity_derivative = dynamics.get_vehicle_angular_velocity_derivative(
      state_.kinematics.angular_velocity, model_.get_vehicle_inertia(),
      model_.get_torque_by_motors(state_.actuators.motor_angular_velocity,
                                  state_.actuators.motor_angular_acceleration),
      model_.get_aerodynamic_moment(state_.kinematics.angular_velocity),
      model_.get_stochastic_force(), state_.dynamics.torque);

  EXPECT_LT(angular_velocity_derivative(0), 0.0);
  EXPECT_EQ(angular_velocity_derivative(1), 0.0);
  EXPECT_EQ(angular_velocity_derivative(2), 0.0);

  // Test accelerate vehicle in x axis:
  // - Decelerate motors 0 and 3
  // - Accelerate motors 1 and 2
  state_.actuators.motor_angular_velocity(0)     = min_speed;
  state_.actuators.motor_angular_velocity(1)     = max_speed;
  state_.actuators.motor_angular_velocity(2)     = max_speed;
  state_.actuators.motor_angular_velocity(3)     = min_speed;
  state_.actuators.motor_angular_acceleration(0) = -motor_accel;
  state_.actuators.motor_angular_acceleration(1) = motor_accel;
  state_.actuators.motor_angular_acceleration(2) = motor_accel;
  state_.actuators.motor_angular_acceleration(3) = -motor_accel;

  angular_velocity_derivative = dynamics.get_vehicle_angular_velocity_derivative(
      state_.kinematics.angular_velocity, model_.get_vehicle_inertia(),
      model_.get_torque_by_motors(state_.actuators.motor_angular_velocity,
                                  state_.actuators.motor_angular_acceleration),
      model_.get_aerodynamic_moment(state_.kinematics.angular_velocity),
      model_.get_stochastic_force(), state_.dynamics.torque);

  EXPECT_GT(angular_velocity_derivative(0), 0.0);
  EXPECT_EQ(angular_velocity_derivative(1), 0.0);
  EXPECT_EQ(angular_velocity_derivative(2), 0.0);

  // Test decelerate vehicle in y axis:
  // - Accelerate motors 0 and 1
  // - Decelerate motors 2 and 3
  state_.actuators.motor_angular_velocity(0)     = max_speed;
  state_.actuators.motor_angular_velocity(1)     = max_speed;
  state_.actuators.motor_angular_velocity(2)     = min_speed;
  state_.actuators.motor_angular_velocity(3)     = min_speed;
  state_.actuators.motor_angular_acceleration(0) = motor_accel;
  state_.actuators.motor_angular_acceleration(1) = motor_accel;
  state_.actuators.motor_angular_acceleration(2) = -motor_accel;
  state_.actuators.motor_angular_acceleration(3) = -motor_accel;

  angular_velocity_derivative = dynamics.get_vehicle_angular_velocity_derivative(
      state_.kinematics.angular_velocity, model_.get_vehicle_inertia(),
      model_.get_torque_by_motors(state_.actuators.motor_angular_velocity,
                                  state_.actuators.motor_angular_acceleration),
      model_.get_aerodynamic_moment(state_.kinematics.angular_velocity),
      model_.get_stochastic_force(), state_.dynamics.torque);

  EXPECT_EQ(angular_velocity_derivative(0), 0.0);
  EXPECT_LT(angular_velocity_derivative(1), 0.0);
  EXPECT_EQ(angular_velocity_derivative(2), 0.0);

  // Test accelerate vehicle in y axis:
  // - Decelerate motors 0 and 1
  // - Accelerate motors 2 and 3
  state_.actuators.motor_angular_velocity(0)     = min_speed;
  state_.actuators.motor_angular_velocity(1)     = min_speed;
  state_.actuators.motor_angular_velocity(2)     = max_speed;
  state_.actuators.motor_angular_velocity(3)     = max_speed;
  state_.actuators.motor_angular_acceleration(0) = -motor_accel;
  state_.actuators.motor_angular_acceleration(1) = -motor_accel;
  state_.actuators.motor_angular_acceleration(2) = motor_accel;
  state_.actuators.motor_angular_acceleration(3) = motor_accel;

  angular_velocity_derivative = dynamics.get_vehicle_angular_velocity_derivative(
      state_.kinematics.angular_velocity, model_.get_vehicle_inertia(),
      model_.get_torque_by_motors(state_.actuators.motor_angular_velocity,
                                  state_.actuators.motor_angular_acceleration),
      model_.get_aerodynamic_moment(state_.kinematics.angular_velocity),
      model_.get_stochastic_force(), state_.dynamics.torque);

  EXPECT_EQ(angular_velocity_derivative(0), 0.0);
  EXPECT_GT(angular_velocity_derivative(1), 0.0);
  EXPECT_EQ(angular_velocity_derivative(2), 0.0);

  // Test decelerate vehicle in z axis:
  // - Decelerate motors 0 and 2
  // - Accelerate motors 1 and 3
  state_.actuators.motor_angular_velocity(0)     = min_speed;
  state_.actuators.motor_angular_velocity(1)     = max_speed;
  state_.actuators.motor_angular_velocity(2)     = min_speed;
  state_.actuators.motor_angular_velocity(3)     = max_speed;
  state_.actuators.motor_angular_acceleration(0) = -motor_accel;
  state_.actuators.motor_angular_acceleration(1) = motor_accel;
  state_.actuators.motor_angular_acceleration(2) = -motor_accel;
  state_.actuators.motor_angular_acceleration(3) = motor_accel;

  angular_velocity_derivative = dynamics.get_vehicle_angular_velocity_derivative(
      state_.kinematics.angular_velocity, model_.get_vehicle_inertia(),
      model_.get_torque_by_motors(state_.actuators.motor_angular_velocity,
                                  state_.actuators.motor_angular_acceleration),
      model_.get_aerodynamic_moment(state_.kinematics.angular_velocity),
      model_.get_stochastic_force(), state_.dynamics.torque);

  EXPECT_EQ(angular_velocity_derivative(0), 0.0);
  EXPECT_EQ(angular_velocity_derivative(1), 0.0);
  EXPECT_LT(angular_velocity_derivative(2), 0.0);

  // Test accelerate vehicle in z axis:
  // - Accelerate motors 0 and 2
  // - Decelerate motors 1 and 3
  state_.actuators.motor_angular_velocity(0)     = max_speed;
  state_.actuators.motor_angular_velocity(1)     = min_speed;
  state_.actuators.motor_angular_velocity(2)     = max_speed;
  state_.actuators.motor_angular_velocity(3)     = min_speed;
  state_.actuators.motor_angular_acceleration(0) = motor_accel;
  state_.actuators.motor_angular_acceleration(1) = -motor_accel;
  state_.actuators.motor_angular_acceleration(2) = motor_accel;
  state_.actuators.motor_angular_acceleration(3) = -motor_accel;

  angular_velocity_derivative = dynamics.get_vehicle_angular_velocity_derivative(
      state_.kinematics.angular_velocity, model_.get_vehicle_inertia(),
      model_.get_torque_by_motors(state_.actuators.motor_angular_velocity,
                                  state_.actuators.motor_angular_acceleration),
      model_.get_aerodynamic_moment(state_.kinematics.angular_velocity),
      model_.get_stochastic_force(), state_.dynamics.torque);

  EXPECT_EQ(angular_velocity_derivative(0), 0.0);
  EXPECT_EQ(angular_velocity_derivative(1), 0.0);
  EXPECT_GT(angular_velocity_derivative(2), 0.0);
}

Eigen::Quaterniond euler_to_quaternion(double roll, double pitch, double yaw) {
  // Calculate half angles
  double roll_half  = roll * 0.5;
  double pitch_half = pitch * 0.5;
  double yaw_half   = yaw * 0.5;

  // Calculate the sine and cosine of the half angles
  double sr = sin(roll_half);
  double cr = cos(roll_half);
  double sp = sin(pitch_half);
  double cp = cos(pitch_half);
  double sy = sin(yaw_half);
  double cy = cos(yaw_half);

  // Calculate the quaternion components
  double w = cr * cp * cy + sr * sp * sy;
  double x = sr * cp * cy - cr * sp * sy;
  double y = cr * sp * cy + sr * cp * sy;
  double z = cr * cp * sy - sr * sp * cy;

  // Create the Quaternion object
  return Eigen::Quaterniond(w, x, y, z).normalized();
}

TEST(Dynamics, protected_methods_get_thrust_force_in_earth_frame) {
  Model<double, 4> model_ = get_model();
  State<double, 4> state_ = State<double, 4>();

  PublicDynamics<double, 4> dynamics(model_, state_);
  Eigen::Vector3d thrust_force = Eigen::Vector3d::Zero();

  // Initialize variables
  const double max_speed                      = model_.get_motors()[0].max_speed;
  state_.actuators.motor_angular_velocity     = Eigen::Vector4d::Ones() * max_speed;
  state_.actuators.motor_angular_acceleration = Eigen::Vector4d::Zero();
  double thrust_coefficient                   = model_.get_motors()[0].thrust_coefficient;
  double force_in_body_frame                  = 4.0 * thrust_coefficient * std::pow(max_speed, 2);
  double expected_force_in_earth              = force_in_body_frame;
  double epsilon                              = 1e-6;
  double roll                                 = 0.0;
  double pitch                                = 0.0;
  double yaw                                  = 0.0;
  state_.kinematics.orientation               = euler_to_quaternion(roll, pitch, yaw);
  double angle                                = M_PI_4 * 0.5;

  EXPECT_NO_THROW(
      dynamics.get_thrust_force_in_earth_frame(state_.actuators.motor_angular_velocity));

  // Vehicle has a X-Configuration, so force is always applied in the Z axis of
  // body frame So, the force in earth frame depends on the vehicle orientation.

  // Test negative force in Y axis:
  roll                          = angle;
  pitch                         = 0.0;
  yaw                           = 0.0;
  state_.kinematics.orientation = euler_to_quaternion(roll, pitch, yaw);
  dynamics.set_state(state_);
  thrust_force = dynamics.get_thrust_force_in_earth_frame(state_.actuators.motor_angular_velocity);

  // Check force
  EXPECT_EQ(thrust_force(0), 0.0);
  EXPECT_LT(thrust_force(1), 0.0);
  EXPECT_GT(thrust_force(2),
            0.0);  // Vehicle is at maximum speed, so force compensate gravity

  EXPECT_NEAR(thrust_force(0), 0.0, epsilon);
  EXPECT_NEAR(thrust_force(1), -std::sin(roll) * force_in_body_frame, epsilon);
  EXPECT_NEAR(thrust_force(2), std::cos(roll) * force_in_body_frame, epsilon);

  // Test positive force in Y axis:
  roll                          = -1.0 * angle;
  pitch                         = 0.0;
  yaw                           = 0.0;
  state_.kinematics.orientation = euler_to_quaternion(roll, pitch, yaw);

  dynamics.set_state(state_);
  thrust_force = dynamics.get_thrust_force_in_earth_frame(state_.actuators.motor_angular_velocity);

  EXPECT_EQ(thrust_force(0), 0.0);
  EXPECT_GT(thrust_force(1), 0.0);
  EXPECT_GT(thrust_force(2), 0.0);

  EXPECT_NEAR(thrust_force(0), 0.0, epsilon);
  EXPECT_NEAR(thrust_force(1), -std::sin(roll) * force_in_body_frame, epsilon);
  EXPECT_NEAR(thrust_force(2), std::cos(roll) * force_in_body_frame, epsilon);

  // Test positive force in X axis:
  roll                          = 0.0;
  pitch                         = angle;
  yaw                           = 0.0;
  state_.kinematics.orientation = euler_to_quaternion(roll, pitch, yaw);

  dynamics.set_state(state_);
  thrust_force = dynamics.get_thrust_force_in_earth_frame(state_.actuators.motor_angular_velocity);

  EXPECT_GT(thrust_force(0), 0.0);
  EXPECT_EQ(thrust_force(1), 0.0);
  EXPECT_GT(thrust_force(2), 0.0);

  EXPECT_NEAR(thrust_force(0), std::sin(pitch) * force_in_body_frame, epsilon);
  EXPECT_NEAR(thrust_force(1), 0.0, epsilon);
  EXPECT_NEAR(thrust_force(2), std::cos(pitch) * force_in_body_frame, epsilon);

  // Test negative force in X axis:
  roll                          = 0.0;
  pitch                         = -1.0 * angle;
  yaw                           = 0.0;
  state_.kinematics.orientation = euler_to_quaternion(roll, pitch, yaw);

  dynamics.set_state(state_);
  thrust_force = dynamics.get_thrust_force_in_earth_frame(state_.actuators.motor_angular_velocity);

  EXPECT_LT(thrust_force(0), 0.0);
  EXPECT_EQ(thrust_force(1), 0.0);
  EXPECT_GT(thrust_force(2), 0.0);

  EXPECT_NEAR(thrust_force(0), std::sin(pitch) * force_in_body_frame, epsilon);
  EXPECT_NEAR(thrust_force(1), 0.0, epsilon);
  EXPECT_NEAR(thrust_force(2), std::cos(pitch) * force_in_body_frame, epsilon);

  // Test positive force in X and in Y axis:
  // x rotation = - degrees
  // y rotation = + degrees
  roll                          = -angle;
  pitch                         = angle;
  yaw                           = 0.0;
  state_.kinematics.orientation = euler_to_quaternion(roll, pitch, yaw);

  dynamics.set_state(state_);
  thrust_force = dynamics.get_thrust_force_in_earth_frame(state_.actuators.motor_angular_velocity);

  EXPECT_GT(thrust_force(0), 0.0);
  EXPECT_GT(thrust_force(1), 0.0);
  EXPECT_GT(thrust_force(2), 0.0);

  // Test negative force in X and Y axis:
  // x rotation = + degrees
  // y rotation = - degrees
  roll                          = angle;
  pitch                         = -angle;
  yaw                           = 0.0;
  state_.kinematics.orientation = euler_to_quaternion(roll, pitch, yaw);

  dynamics.set_state(state_);
  thrust_force = dynamics.get_thrust_force_in_earth_frame(state_.actuators.motor_angular_velocity);

  EXPECT_LT(thrust_force(0), 0.0);
  EXPECT_LT(thrust_force(1), 0.0);
  EXPECT_GT(thrust_force(2), 0.0);

  // Test negative force in X and positive in Y axis:
  // x rotation = - degrees
  // y rotation = - degrees
  roll                          = -angle;
  pitch                         = -angle;
  yaw                           = 0.0;
  state_.kinematics.orientation = euler_to_quaternion(roll, pitch, yaw);

  dynamics.set_state(state_);
  thrust_force = dynamics.get_thrust_force_in_earth_frame(state_.actuators.motor_angular_velocity);

  EXPECT_LT(thrust_force(0), 0.0);
  EXPECT_GT(thrust_force(1), 0.0);
  EXPECT_GT(thrust_force(2), 0.0);

  // Test positive force in X and negative in Y axis:
  // x rotation = + degrees
  // y rotation = + degrees
  roll                          = angle;
  pitch                         = angle;
  yaw                           = 0.0;
  state_.kinematics.orientation = euler_to_quaternion(roll, pitch, yaw);

  dynamics.set_state(state_);
  thrust_force = dynamics.get_thrust_force_in_earth_frame(state_.actuators.motor_angular_velocity);

  EXPECT_GT(thrust_force(0), 0.0);
  EXPECT_LT(thrust_force(1), 0.0);
  EXPECT_GT(thrust_force(2), 0.0);

  // Test max force in Z axis:
  state_.kinematics.orientation = Eigen::Quaterniond::Identity();
  state_.kinematics.orientation.normalize();
  state_.actuators.motor_angular_velocity =
      Eigen::Vector4d(max_speed, max_speed, max_speed, max_speed);

  dynamics.set_state(state_);
  thrust_force = dynamics.get_thrust_force_in_earth_frame(state_.actuators.motor_angular_velocity);

  EXPECT_EQ(thrust_force(0), 0.0);
  EXPECT_EQ(thrust_force(1), 0.0);
  EXPECT_GT(thrust_force(2), 0.0);

  // Check magnitude of force
  EXPECT_EQ(thrust_force(2), force_in_body_frame);

  // Test zero force in Z axis:
  state_.kinematics.orientation = Eigen::Quaterniond::Identity();
  state_.kinematics.orientation.normalize();
  state_.actuators.motor_angular_velocity = Eigen::Vector4d(0.0, 0.0, 0.0, 0.0);

  dynamics.set_state(state_);
  thrust_force = dynamics.get_thrust_force_in_earth_frame(state_.actuators.motor_angular_velocity);

  EXPECT_EQ(thrust_force(0), 0.0);
  EXPECT_EQ(thrust_force(1), 0.0);
  EXPECT_EQ(thrust_force(2), 0.0);
}

TEST(Dynamics, protected_methods_get_vehicle_linear_velocity_derivative) {
  Model<double, 4> model_ = get_model();
  State<double, 4> state_ = State<double, 4>();

  PublicDynamics<double, 4> dynamics(model_, state_);
  Eigen::Vector3d external_force             = Eigen::Vector3d::Zero();
  Eigen::Vector3d linear_velocity_derivative = Eigen::Vector3d::Ones();

  // Initialize actuators state
  const double max_speed                      = model_.get_motors()[0].max_speed;
  state_.actuators.motor_angular_velocity     = Eigen::Vector4d::Ones() * max_speed;
  state_.actuators.motor_angular_acceleration = Eigen::Vector4d::Zero();
  double epsilon                              = 1e-6;
  double roll                                 = 0.0;
  double pitch                                = 0.0;
  double yaw                                  = 0.0;
  state_.kinematics.orientation               = euler_to_quaternion(roll, pitch, yaw);
  double angle                                = M_PI_4 * 0.5;

  EXPECT_NO_THROW(dynamics.get_vehicle_linear_velocity_derivative(
      model_.get_mass(),
      dynamics.get_thrust_force_in_earth_frame(state_.actuators.motor_angular_velocity),
      model_.get_drag_force(state_.kinematics.linear_velocity), model_.get_gravity(),
      model_.get_stochastic_force(), external_force, state_.dynamics.force));

  // Vehicle has a X-Configuration, so force is always applied in the Z axis.
  // So, the linear velocity derivative depends on the vehicle orientation.

  // Test negative vehicle velocity in Y axis:
  roll                          = angle;
  pitch                         = 0.0;
  yaw                           = 0.0;
  state_.kinematics.orientation = euler_to_quaternion(roll, pitch, yaw);

  dynamics.set_state(state_);
  linear_velocity_derivative = dynamics.get_vehicle_linear_velocity_derivative(
      model_.get_mass(),
      dynamics.get_thrust_force_in_earth_frame(state_.actuators.motor_angular_velocity),
      model_.get_drag_force(state_.kinematics.linear_velocity), model_.get_gravity(),
      model_.get_stochastic_force(), external_force, state_.dynamics.force);

  EXPECT_EQ(linear_velocity_derivative(0), 0.0);
  EXPECT_LT(linear_velocity_derivative(1), 0.0);
  EXPECT_GT(linear_velocity_derivative(2), 0.0);

  // Test positive vehicle velocity in Y axis:
  roll                          = -1.0 * angle;
  pitch                         = 0.0;
  yaw                           = 0.0;
  state_.kinematics.orientation = euler_to_quaternion(roll, pitch, yaw);

  dynamics.set_state(state_);
  linear_velocity_derivative = dynamics.get_vehicle_linear_velocity_derivative(
      model_.get_mass(),
      dynamics.get_thrust_force_in_earth_frame(state_.actuators.motor_angular_velocity),
      model_.get_drag_force(state_.kinematics.linear_velocity), model_.get_gravity(),
      model_.get_stochastic_force(), external_force, state_.dynamics.force);

  EXPECT_EQ(linear_velocity_derivative(0), 0.0);
  EXPECT_GT(linear_velocity_derivative(1), 0.0);
  EXPECT_GT(linear_velocity_derivative(2), 0.0);

  // Test positive vehicle velocity in X axis:
  roll                          = 0.0;
  pitch                         = angle;
  yaw                           = 0.0;
  state_.kinematics.orientation = euler_to_quaternion(roll, pitch, yaw);

  dynamics.set_state(state_);
  linear_velocity_derivative = dynamics.get_vehicle_linear_velocity_derivative(
      model_.get_mass(),
      dynamics.get_thrust_force_in_earth_frame(state_.actuators.motor_angular_velocity),
      model_.get_drag_force(state_.kinematics.linear_velocity), model_.get_gravity(),
      model_.get_stochastic_force(), external_force, state_.dynamics.force);

  EXPECT_GT(linear_velocity_derivative(0), 0.0);
  EXPECT_EQ(linear_velocity_derivative(1), 0.0);
  EXPECT_GT(linear_velocity_derivative(2), 0.0);

  // Test negative vehicle velocity in X axis:
  roll                          = 0.0;
  pitch                         = -1.0 * angle;
  yaw                           = 0.0;
  state_.kinematics.orientation = euler_to_quaternion(roll, pitch, yaw);

  dynamics.set_state(state_);
  linear_velocity_derivative = dynamics.get_vehicle_linear_velocity_derivative(
      model_.get_mass(),
      dynamics.get_thrust_force_in_earth_frame(state_.actuators.motor_angular_velocity),
      model_.get_drag_force(state_.kinematics.linear_velocity), model_.get_gravity(),
      model_.get_stochastic_force(), external_force, state_.dynamics.force);

  EXPECT_LT(linear_velocity_derivative(0), 0.0);
  EXPECT_EQ(linear_velocity_derivative(1), 0.0);
  EXPECT_GT(linear_velocity_derivative(2), 0.0);

  // Test vehicle velocity in Z axis:
  state_.kinematics.orientation = Eigen::Quaterniond::Identity();

  // Test decelerate vehicle in z axis:
  // - Decelerate all motors
  state_.actuators.motor_angular_velocity(0) = 0.0;
  state_.actuators.motor_angular_velocity(1) = 0.0;
  state_.actuators.motor_angular_velocity(2) = 0.0;
  state_.actuators.motor_angular_velocity(3) = 0.0;

  dynamics.set_state(state_);
  linear_velocity_derivative = dynamics.get_vehicle_linear_velocity_derivative(
      model_.get_mass(),
      dynamics.get_thrust_force_in_earth_frame(state_.actuators.motor_angular_velocity),
      model_.get_drag_force(state_.kinematics.linear_velocity), model_.get_gravity(),
      model_.get_stochastic_force(), external_force, state_.dynamics.force);

  EXPECT_EQ(linear_velocity_derivative(0), 0.0);
  EXPECT_EQ(linear_velocity_derivative(1), 0.0);
  EXPECT_LE(linear_velocity_derivative(2), 0.0);

  // Test accelerate vehicle in z axis:
  // - Accelerate all motors
  state_.actuators.motor_angular_velocity(0) = max_speed;
  state_.actuators.motor_angular_velocity(1) = max_speed;
  state_.actuators.motor_angular_velocity(2) = max_speed;
  state_.actuators.motor_angular_velocity(3) = max_speed;

  dynamics.set_state(state_);
  linear_velocity_derivative = dynamics.get_vehicle_linear_velocity_derivative(
      model_.get_mass(),
      dynamics.get_thrust_force_in_earth_frame(state_.actuators.motor_angular_velocity),
      model_.get_drag_force(state_.kinematics.linear_velocity), model_.get_gravity(),
      model_.get_stochastic_force(), external_force, state_.dynamics.force);

  EXPECT_EQ(linear_velocity_derivative(0), 0.0);
  EXPECT_EQ(linear_velocity_derivative(1), 0.0);
  EXPECT_GT(linear_velocity_derivative(2), 0.0);
}

int main(int argc, char *argv[]) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
