/*!*******************************************************************************************
 *  \file       model_gtest.hpp
 *  \brief      Model class unit tests.
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
#include "multirotor_dynamic_model/model.hpp"

namespace multirotor::model {

TEST(MotorParams, constructor_default_template) {
  MotorParams motor;
  motor.motor_rotation_direction = 1;
  motor.thrust_coefficient       = 1.0;
  motor.torque_coefficient       = 2.0;
  motor.pose.translation()       = Eigen::Vector3d(1.0, 2.0, 3.0);
  motor.min_speed                = 3.0;
  motor.max_speed                = 4.0;
  motor.time_constant            = 5.0;
  motor.rotational_inertia       = 6.0;

  EXPECT_EQ(motor.motor_rotation_direction, 1);
  EXPECT_EQ(motor.thrust_coefficient, 1.0);
  EXPECT_EQ(motor.torque_coefficient, 2.0);
  EXPECT_EQ(motor.pose.translation(), Eigen::Vector3d(1.0, 2.0, 3.0));
  EXPECT_EQ(motor.min_speed, 3.0);
  EXPECT_EQ(motor.max_speed, 4.0);
  EXPECT_EQ(motor.time_constant, 5.0);
  EXPECT_EQ(motor.rotational_inertia, 6.0);
}

TEST(MotorParams, constructor_float_precision) {
  MotorParams<float> motor;
  motor.motor_rotation_direction = 1;
  motor.thrust_coefficient       = 1.0f;
  motor.torque_coefficient       = 2.0f;
  motor.pose.translation()       = Eigen::Vector3f(1.0f, 2.0f, 3.0f);
  motor.min_speed                = 3.0f;
  motor.max_speed                = 4.0f;
  motor.time_constant            = 5.0f;
  motor.rotational_inertia       = 6.0f;

  EXPECT_EQ(motor.motor_rotation_direction, 1);
  EXPECT_EQ(motor.thrust_coefficient, 1.0f);
  EXPECT_EQ(motor.torque_coefficient, 2.0f);
  EXPECT_EQ(motor.pose.translation(), Eigen::Vector3f(1.0f, 2.0f, 3.0f));
  EXPECT_EQ(motor.min_speed, 3.0f);
  EXPECT_EQ(motor.max_speed, 4.0f);
  EXPECT_EQ(motor.time_constant, 5.0f);
  EXPECT_EQ(motor.rotational_inertia, 6.0f);
}

TEST(MotorParams, constructor_double_precision) {
  MotorParams<double> motor;
  motor.motor_rotation_direction = 1;
  motor.thrust_coefficient       = 1.0;
  motor.torque_coefficient       = 2.0;
  motor.pose.translation()       = Eigen::Vector3d(1.0, 2.0, 3.0);
  motor.min_speed                = 3.0;
  motor.max_speed                = 4.0;
  motor.time_constant            = 5.0;
  motor.rotational_inertia       = 6.0;

  EXPECT_EQ(motor.motor_rotation_direction, 1);
  EXPECT_EQ(motor.thrust_coefficient, 1.0);
  EXPECT_EQ(motor.torque_coefficient, 2.0);
  EXPECT_EQ(motor.pose.translation(), Eigen::Vector3d(1.0, 2.0, 3.0));
  EXPECT_EQ(motor.min_speed, 3.0);
  EXPECT_EQ(motor.max_speed, 4.0);
  EXPECT_EQ(motor.time_constant, 5.0);
  EXPECT_EQ(motor.rotational_inertia, 6.0);
}

TEST(ModelParams, constructor_default_template) {
  const int num_motors = 4;
  ModelParams model;
  MotorParams motor;
  motor.motor_rotation_direction = 1;
  motor.thrust_coefficient       = 1.0;
  motor.torque_coefficient       = 2.0;
  motor.pose.translation()       = Eigen::Vector3d(1.0, 2.0, 3.0);
  motor.min_speed                = 3.0;
  motor.max_speed                = 4.0;
  motor.time_constant            = 5.0;
  motor.rotational_inertia       = 6.0;

  for (int i = 0; i < num_motors; i++) {
    model.motors_params[i] = motor;
  }

  model.vehicle_mass                          = 10.0;
  model.vehicle_inertia                       = Eigen::Matrix3d::Identity();
  model.vehicle_drag_coefficient              = 0.1;
  model.vehicle_aero_moment_coefficient       = Eigen::Matrix3d::Identity();
  model.gravity                               = Eigen::Vector3d(0.0, 0.0, -9.81);
  model.moment_process_noise_auto_correlation = 0.2;
  model.force_process_noise_auto_correlation  = 0.3;

  EXPECT_EQ(model.motors_params.size(), num_motors);
  EXPECT_EQ(model.vehicle_mass, 10.0);
  EXPECT_EQ(model.vehicle_inertia, Eigen::Matrix3d::Identity());
  EXPECT_EQ(model.vehicle_drag_coefficient, 0.1);
  EXPECT_EQ(model.vehicle_aero_moment_coefficient, Eigen::Matrix3d::Identity());
  EXPECT_EQ(model.gravity, Eigen::Vector3d(0.0, 0.0, -9.81));
  EXPECT_EQ(model.moment_process_noise_auto_correlation, 0.2);
  EXPECT_EQ(model.force_process_noise_auto_correlation, 0.3);
}

TEST(ModelParams, constructor_float_precision) {
  const int num_motors = 4;
  ModelParams<float, num_motors> model;
  MotorParams<float> motor;
  motor.motor_rotation_direction = 1;
  motor.thrust_coefficient       = 1.0f;
  motor.torque_coefficient       = 2.0f;
  motor.pose.translation()       = Eigen::Vector3f(1.0f, 2.0f, 3.0f);
  motor.min_speed                = 3.0f;
  motor.max_speed                = 4.0f;
  motor.time_constant            = 5.0f;
  motor.rotational_inertia       = 6.0f;

  for (int i = 0; i < num_motors; i++) {
    model.motors_params[i] = motor;
  }

  model.vehicle_mass                          = 10.0f;
  model.vehicle_inertia                       = Eigen::Matrix3f::Identity();
  model.vehicle_drag_coefficient              = 0.1f;
  model.vehicle_aero_moment_coefficient       = Eigen::Matrix3f::Identity();
  model.gravity                               = Eigen::Vector3f(0.0f, 0.0f, -9.81f);
  model.moment_process_noise_auto_correlation = 0.2f;
  model.force_process_noise_auto_correlation  = 0.3f;

  EXPECT_EQ(model.motors_params.size(), num_motors);
  EXPECT_EQ(model.vehicle_mass, 10.0f);
  EXPECT_EQ(model.vehicle_inertia, Eigen::Matrix3f::Identity());
  EXPECT_EQ(model.vehicle_drag_coefficient, 0.1f);
  EXPECT_EQ(model.vehicle_aero_moment_coefficient, Eigen::Matrix3f::Identity());
  EXPECT_EQ(model.gravity, Eigen::Vector3f(0.0f, 0.0f, -9.81f));
  EXPECT_EQ(model.moment_process_noise_auto_correlation, 0.2f);
  EXPECT_EQ(model.force_process_noise_auto_correlation, 0.3f);
}

TEST(ModelParams, constructor_double_precision) {
  const int num_motors = 4;
  ModelParams<double, num_motors> model;

  MotorParams<double> motor;
  motor.motor_rotation_direction = 1;
  motor.thrust_coefficient       = 1.0;
  motor.torque_coefficient       = 2.0;
  motor.pose.translation()       = Eigen::Vector3d(1.0, 2.0, 3.0);
  motor.min_speed                = 3.0;
  motor.max_speed                = 4.0;
  motor.time_constant            = 5.0;
  motor.rotational_inertia       = 6.0;

  for (int i = 0; i < num_motors; i++) {
    model.motors_params[i] = motor;
  }

  model.vehicle_mass                          = 10.0;
  model.vehicle_inertia                       = Eigen::Matrix3d::Identity();
  model.vehicle_drag_coefficient              = 0.1;
  model.vehicle_aero_moment_coefficient       = Eigen::Matrix3d::Identity();
  model.gravity                               = Eigen::Vector3d(0.0, 0.0, -9.81);
  model.moment_process_noise_auto_correlation = 0.2;
  model.force_process_noise_auto_correlation  = 0.3;

  EXPECT_EQ(model.motors_params.size(), num_motors);
  EXPECT_EQ(model.vehicle_mass, 10.0);
  EXPECT_EQ(model.vehicle_inertia, Eigen::Matrix3d::Identity());
  EXPECT_EQ(model.vehicle_drag_coefficient, 0.1);
  EXPECT_EQ(model.vehicle_aero_moment_coefficient, Eigen::Matrix3d::Identity());
  EXPECT_EQ(model.gravity, Eigen::Vector3d(0.0, 0.0, -9.81));
  EXPECT_EQ(model.moment_process_noise_auto_correlation, 0.2);
  EXPECT_EQ(model.force_process_noise_auto_correlation, 0.3);
}

TEST(Model, constructor_default) { EXPECT_NO_THROW(Model model = Model()); }

TEST(Model, create_quadrotor_x_config) {
  const double thrust_coefficient = 2.0;
  const double torque_coefficient = 1.0;
  const double x_dist             = 0.2;
  const double y_dist             = 0.3;
  const double min_speed          = 0.0;
  const double max_speed          = 100.0;
  const double time_constant      = 0.1;
  const double rotational_inertia = 0.5;

  std::vector<MotorParams<double>> motors = Model<double, 4>::create_quadrotor_x_config(
      thrust_coefficient, torque_coefficient, x_dist, y_dist, min_speed, max_speed, time_constant,
      rotational_inertia);

  // Check the motors
  EXPECT_EQ(motors.size(), 4);

  // Check the motor parameters
  for (auto motor : motors) {
    EXPECT_EQ(motor.thrust_coefficient, thrust_coefficient);
    EXPECT_EQ(motor.torque_coefficient, torque_coefficient);
    EXPECT_EQ(motor.min_speed, min_speed);
    EXPECT_EQ(motor.max_speed, max_speed);
    EXPECT_EQ(motor.time_constant, time_constant);
    EXPECT_EQ(motor.rotational_inertia, rotational_inertia);
  }

  // Check the motor positions
  EXPECT_EQ(motors[0].pose.translation().x(), x_dist);
  EXPECT_EQ(motors[0].pose.translation().y(), -y_dist);
  EXPECT_EQ(motors[1].pose.translation().x(), x_dist);
  EXPECT_EQ(motors[1].pose.translation().y(), y_dist);
  EXPECT_EQ(motors[2].pose.translation().x(), -x_dist);
  EXPECT_EQ(motors[2].pose.translation().y(), y_dist);
  EXPECT_EQ(motors[3].pose.translation().x(), -x_dist);
  EXPECT_EQ(motors[3].pose.translation().y(), -y_dist);
}

TEST(Model, create_quadrotor_plus_config) {
  const double thrust_coefficient = 2.0;
  const double torque_coefficient = 1.0;
  const double x_dist             = 0.2;
  const double y_dist             = 0.3;
  const double min_speed          = 0.0;
  const double max_speed          = 100.0;
  const double time_constant      = 0.1;
  const double rotational_inertia = 0.5;

  std::vector<MotorParams<double>> motors = Model<double, 4>::create_quadrotor_plus_config(
      thrust_coefficient, torque_coefficient, x_dist, y_dist, min_speed, max_speed, time_constant,
      rotational_inertia);

  // Check the motors
  EXPECT_EQ(motors.size(), 4);

  // Check the motor parameters
  for (auto motor : motors) {
    EXPECT_EQ(motor.thrust_coefficient, thrust_coefficient);
    EXPECT_EQ(motor.torque_coefficient, torque_coefficient);
    EXPECT_EQ(motor.min_speed, min_speed);
    EXPECT_EQ(motor.max_speed, max_speed);
    EXPECT_EQ(motor.time_constant, time_constant);
    EXPECT_EQ(motor.rotational_inertia, rotational_inertia);
  }

  // Check the motor positions
  EXPECT_EQ(motors[0].pose.translation().x(), x_dist);
  EXPECT_EQ(motors[0].pose.translation().y(), 0.0);
  EXPECT_EQ(motors[1].pose.translation().x(), 0.0);
  EXPECT_EQ(motors[1].pose.translation().y(), y_dist);
  EXPECT_EQ(motors[2].pose.translation().x(), -x_dist);
  EXPECT_EQ(motors[2].pose.translation().y(), 0.0);
  EXPECT_EQ(motors[3].pose.translation().x(), 0.0);
  EXPECT_EQ(motors[3].pose.translation().y(), -y_dist);
}

TEST(Model, create_hexacopter_config) {
  const double thrust_coefficient = 2.0;
  const double torque_coefficient = 1.0;
  const double radius             = 0.6;
  const double min_speed          = 0.0;
  const double max_speed          = 100.0;
  const double time_constant      = 0.1;
  const double rotational_inertia = 0.5;

  std::vector<MotorParams<double>> motors = Model<double, 6>::create_hexacopter_config(
      thrust_coefficient, torque_coefficient, radius, min_speed, max_speed, time_constant,
      rotational_inertia);

  // Check the motors
  EXPECT_EQ(motors.size(), 6);

  // Check the motor parameters
  for (auto motor : motors) {
    EXPECT_EQ(motor.thrust_coefficient, thrust_coefficient);
    EXPECT_EQ(motor.torque_coefficient, torque_coefficient);
    EXPECT_EQ(motor.min_speed, min_speed);
    EXPECT_EQ(motor.max_speed, max_speed);
    EXPECT_EQ(motor.time_constant, time_constant);
    EXPECT_EQ(motor.rotational_inertia, rotational_inertia);
  }

  // Check the motor positions
  const double angle  = M_PI / 3.0;
  const double x_dist = radius * std::cos(angle);
  const double y_dist = radius * std::sin(angle);
  EXPECT_EQ(motors[0].pose.translation().x(), radius);
  EXPECT_EQ(motors[0].pose.translation().y(), 0.0);
  EXPECT_EQ(motors[1].pose.translation().x(), x_dist);
  EXPECT_EQ(motors[1].pose.translation().y(), y_dist);
  EXPECT_EQ(motors[2].pose.translation().x(), -x_dist);
  EXPECT_EQ(motors[2].pose.translation().y(), y_dist);
  EXPECT_EQ(motors[3].pose.translation().x(), -radius);
  EXPECT_EQ(motors[3].pose.translation().y(), 0.0);
  EXPECT_EQ(motors[4].pose.translation().x(), -x_dist);
  EXPECT_EQ(motors[4].pose.translation().y(), -y_dist);
  EXPECT_EQ(motors[5].pose.translation().x(), x_dist);
  EXPECT_EQ(motors[5].pose.translation().y(), -y_dist);
}

TEST(Model, compute_mixer_matrix_quadrotor_x) {
  const double thrust_coefficient = 2.0;
  const double torque_coefficient = 1.0;
  const double x_dist             = 0.2;
  const double y_dist             = 0.3;
  const double min_speed          = 0.0;
  const double max_speed          = 100.0;
  const double time_constant      = 0.1;
  const double rotational_inertia = 0.5;

  std::vector<MotorParams<double>> motors = Model<double, 4>::create_quadrotor_x_config(
      thrust_coefficient, torque_coefficient, x_dist, y_dist, min_speed, max_speed, time_constant,
      rotational_inertia);

  // Get the mixing matrix
  Eigen::Matrix4d mixing_matrix = Model<double, 4>::compute_mixer_matrix<4>(motors);

  // Check the matrix
  Eigen::Matrix4d expected_matrix;
  expected_matrix << thrust_coefficient, thrust_coefficient, thrust_coefficient, thrust_coefficient,
      -1.0 * y_dist * thrust_coefficient, y_dist * thrust_coefficient, y_dist * thrust_coefficient,
      -1.0 * y_dist * thrust_coefficient, -1.0 * x_dist * thrust_coefficient,
      -1.0 * x_dist * thrust_coefficient, x_dist * thrust_coefficient, x_dist * thrust_coefficient,
      torque_coefficient, -1.0 * torque_coefficient, torque_coefficient, -1.0 * torque_coefficient;

  EXPECT_TRUE(mixing_matrix.isApprox(expected_matrix));

  // Get the mixing inertia matrix
  Eigen::Matrix<double, 3, 4> mixing_inertia_matrix =
      Model<double, 4>::compute_mixer_inertia_matrix(motors);

  // Check the matrix
  Eigen::Matrix<double, 3, 4> expected_inertia_matrix;
  expected_inertia_matrix << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, rotational_inertia,
      -1.0 * rotational_inertia, rotational_inertia, -1.0 * rotational_inertia;

  EXPECT_TRUE(mixing_inertia_matrix.isApprox(expected_inertia_matrix));

  // Get the mixing matrix 6D
  Eigen::Matrix<double, 6, 4> mixing_matrix_extend =
      Model<double, 4>::compute_mixer_matrix<6>(motors);

  // Check the matrix
  Eigen::Matrix<double, 6, 4> expected_matrix_extend = Eigen::Matrix<double, 6, 4>::Zero();

  // First row is 0.0 as there is no thrust in the x direction
  // Second row is 0.0 as there is no thrust in the y direction
  // Rest is the same as the 4D matrix: expected_matrix_extend(2:, :) =
  // expected_matrix();
  expected_matrix_extend.block<4, 4>(2, 0) = expected_matrix;

  EXPECT_TRUE(mixing_matrix_extend.isApprox(expected_matrix_extend));
}

TEST(Model, compute_mixer_matrix_quadrotor_plus) {
  const double thrust_coefficient = 2.0;
  const double torque_coefficient = 1.0;
  const double x_dist             = 0.2;
  const double y_dist             = 0.3;
  const double min_speed          = 0.0;
  const double max_speed          = 100.0;
  const double time_constant      = 0.1;
  const double rotational_inertia = 0.5;

  std::vector<MotorParams<double>> motors = Model<double, 4>::create_quadrotor_plus_config(
      thrust_coefficient, torque_coefficient, x_dist, y_dist, min_speed, max_speed, time_constant,
      rotational_inertia);

  // Get the mixing matrix
  Eigen::Matrix4d mixing_matrix = Model<double, 4>::compute_mixer_matrix<4>(motors);

  // Check the matrix
  Eigen::Matrix4d expected_matrix;
  expected_matrix << thrust_coefficient, thrust_coefficient, thrust_coefficient, thrust_coefficient,
      0.0, y_dist * thrust_coefficient, 0.0, -1.0 * y_dist * thrust_coefficient,
      -1.0 * x_dist * thrust_coefficient, 0.0, x_dist * thrust_coefficient, 0.0, torque_coefficient,
      -1.0 * torque_coefficient, torque_coefficient, -1.0 * torque_coefficient;

  EXPECT_TRUE(mixing_matrix.isApprox(expected_matrix));

  // Get the mixing inertia matrix
  Eigen::Matrix<double, 3, 4> mixing_inertia_matrix =
      Model<double, 4>::compute_mixer_inertia_matrix(motors);

  // Check the matrix
  Eigen::Matrix<double, 3, 4> expected_inertia_matrix;
  expected_inertia_matrix << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, rotational_inertia,
      -1.0 * rotational_inertia, rotational_inertia, -1.0 * rotational_inertia;

  EXPECT_TRUE(mixing_inertia_matrix.isApprox(expected_inertia_matrix));

  // Get the mixing matrix 6D
  Eigen::Matrix<double, 6, 4> mixing_matrix_extend =
      Model<double, 4>::compute_mixer_matrix<6>(motors);

  // Check the matrix
  Eigen::Matrix<double, 6, 4> expected_matrix_extend = Eigen::Matrix<double, 6, 4>::Zero();

  // First row is 0.0 as there is no thrust in the x direction
  // Second row is 0.0 as there is no thrust in the y direction
  // Rest is the same as the 4D matrix: expected_matrix_extend(2:, :) =
  // expected_matrix();
  expected_matrix_extend.block<4, 4>(2, 0) = expected_matrix;

  EXPECT_TRUE(mixing_matrix_extend.isApprox(expected_matrix_extend));

  // Get the mixing torque
  Eigen::Matrix<double, 3, 4> mixing_torque = mixing_matrix.bottomRows<3>();

  // Check the mixing torque is equal to the last 3 rows of the mixing matrix
  EXPECT_TRUE(mixing_torque.isApprox(mixing_matrix.block<3, 4>(1, 0)));

  // Get the mixing torque 6D
  Eigen::Matrix<double, 3, 4> mixing_torque_extend = mixing_matrix_extend.bottomRows<3>();

  // Check the mixing torque is equal to the last 3 rows of the mixing matrix
  EXPECT_TRUE(mixing_torque_extend.isApprox(mixing_matrix_extend.block<3, 4>(3, 0)));
}

TEST(Model, compute_mixer_matrix_hexacopter) {
  const double thrust_coefficient = 1.0;
  const double torque_coefficient = 2.0;
  const double radius             = 0.5;
  const double min_speed          = 0.0;
  const double max_speed          = 100.0;
  const double time_constant      = 0.1;
  const double rotational_inertia = 0.5;

  std::vector<MotorParams<double>> motors = Model<double, 6>::create_hexacopter_config(
      thrust_coefficient, torque_coefficient, radius, min_speed, max_speed, time_constant,
      rotational_inertia);

  // Get the mixing matrix
  Eigen::Matrix<double, 4, 6> mixing_matrix = Model<double, 6>::compute_mixer_matrix<4>(motors);

  // Check the matrix
  const double angle  = M_PI / 3.0;
  const double x_dist = radius * std::cos(angle);
  const double y_dist = radius * std::sin(angle);

  Eigen::Matrix<double, 4, 6> expected_matrix;
  expected_matrix << thrust_coefficient, thrust_coefficient, thrust_coefficient, thrust_coefficient,
      thrust_coefficient, thrust_coefficient, 0.0, y_dist * thrust_coefficient,
      y_dist * thrust_coefficient, 0.0, -1.0 * y_dist * thrust_coefficient,
      -1.0 * y_dist * thrust_coefficient, -1.0 * radius * thrust_coefficient,
      -1.0 * x_dist * thrust_coefficient, x_dist * thrust_coefficient, radius * thrust_coefficient,
      x_dist * thrust_coefficient, -1.0 * x_dist * thrust_coefficient, torque_coefficient,
      -1.0 * torque_coefficient, torque_coefficient, -1.0 * torque_coefficient, torque_coefficient,
      -1.0 * torque_coefficient;

  EXPECT_TRUE(mixing_matrix.isApprox(expected_matrix));

  // Get the mixing inertia matrix
  Eigen::Matrix<double, 3, 6> mixing_inertia_matrix =
      Model<double, 6>::compute_mixer_inertia_matrix(motors);

  // Check the matrix
  Eigen::Matrix<double, 3, 6> expected_inertia_matrix;
  expected_inertia_matrix << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      rotational_inertia, -1.0 * rotational_inertia, rotational_inertia, -1.0 * rotational_inertia,
      rotational_inertia, -1.0 * rotational_inertia;

  EXPECT_TRUE(mixing_inertia_matrix.isApprox(expected_inertia_matrix));

  // Get the mixing matrix 6D
  Eigen::Matrix<double, 6, 6> mixing_matrix_extend =
      Model<double, 6>::compute_mixer_matrix<6>(motors);

  // Check the matrix
  Eigen::Matrix<double, 6, 6> expected_matrix_extend = Eigen::Matrix<double, 6, 6>::Zero();

  // First row is 0.0 as there is no thrust in the x direction
  // Second row is 0.0 as there is no thrust in the y direction
  // Rest is the same as the 4D matrix: expected_matrix_extend(2:, :) =
  // expected_matrix();
  expected_matrix_extend.block<4, 6>(2, 0) = expected_matrix;

  EXPECT_TRUE(mixing_matrix_extend.isApprox(expected_matrix_extend));

  // Get the mixing torque
  Eigen::Matrix<double, 3, 6> mixing_torque = mixing_matrix.bottomRows<3>();

  // Check the mixing torque is equal to the last 3 rows of the mixing matrix
  EXPECT_TRUE(mixing_torque.isApprox(mixing_matrix.block<3, 6>(1, 0)));

  // Get the mixing torque 6D
  Eigen::Matrix<double, 3, 6> mixing_torque_extend = mixing_matrix_extend.bottomRows<3>();

  // Check the mixing torque is equal to the last 3 rows of the mixing matrix
  EXPECT_TRUE(mixing_torque_extend.isApprox(mixing_matrix_extend.block<3, 6>(3, 0)));
}

TEST(Model, compute_mixer_matrix) {
  const double thrust_coefficient = 1.0;
  const double torque_coefficient = 2.0;
  const double radius             = 0.5;
  const double min_speed          = 0.0;
  const double max_speed          = 100.0;
  const double time_constant      = 0.1;
  const double rotational_inertia = 0.5;

  std::vector<MotorParams<double>> motors = Model<double, 6>::create_hexacopter_config(
      thrust_coefficient, torque_coefficient, radius, min_speed, max_speed, time_constant,
      rotational_inertia);

  // Get the mixing matrix
  Eigen::Matrix<double, 4, 6> mixing_matrix_4d = Model<double, 6>::compute_mixer_matrix<4>(motors);
  Eigen::Matrix<double, 6, 6> mixing_matrix_6d = Model<double, 6>::compute_mixer_matrix<6>(motors);

  // For others dimensions, should fail at compile time
  // EXPECT_DEATH(Model<double, 6>::compute_mixer_matrix<3>(motors), ".*");
}

TEST(Model, instance) {
  const double thrust_coefficient = 2.0;
  const double torque_coefficient = 1.0;
  const double x_dist             = 0.2;
  const double y_dist             = 0.3;
  const double min_speed          = 0.0;
  const double max_speed          = 100.0;
  const double time_constant      = 0.1;
  const double rotational_inertia = 0.5;

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
  // EXPECT_NO_THROW({ Model<double, 4> model = Model<double, 4>(model_params);
  // }); TODO
}

ModelParams<double, 4> get_model_params(const double thrust_coefficient = 2.0,
                                        const double torque_coefficient = 1.0) {
  const double x_dist             = 0.2;
  const double y_dist             = 0.3;
  const double min_speed          = 0.0;
  const double max_speed          = 100.0;
  const double time_constant      = 0.1;
  const double rotational_inertia = 0.5;

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
  return model_params;
}

TEST(Model, instance_fails) {
  const double thrust_coefficient = 2.0;
  const double torque_coefficient = 1.0;
  const double x_dist             = 0.2;
  const double y_dist             = 0.3;
  const double min_speed          = 0.0;
  const double max_speed          = 100.0;
  const double time_constant      = 0.1;
  const double rotational_inertia = 0.5;

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

  // Check vehicle mass
  model_params.vehicle_mass = -1.0;
  EXPECT_FALSE(model.check_params(model_params));
  model_params.vehicle_mass = 1.0;

  // Check vehicle inertia
  model_params.vehicle_inertia = Eigen::Vector3d(-0.0049, 0.0049, 0.0069).asDiagonal();
  EXPECT_FALSE(model.check_params(model_params));
  model_params.vehicle_inertia = Eigen::Vector3d(0.0049, 0.0049, 0.0069).asDiagonal();

  // Check vehicle drag coefficient
  model_params.vehicle_drag_coefficient = -0.1;
  EXPECT_FALSE(model.check_params(model_params));
  model_params.vehicle_drag_coefficient = 0.1;

  // Check vehicle aero moment coefficient
  model_params.vehicle_aero_moment_coefficient = Eigen::Vector3d(-0.003, 0.003, 0.003).asDiagonal();
  EXPECT_FALSE(model.check_params(model_params));
  model_params.vehicle_aero_moment_coefficient = Eigen::Vector3d(0.003, 0.003, 0.003).asDiagonal();

  // Check motors min_speed greater than max_speed
  model_params.motors_params[0].min_speed = 100.0;
  model_params.motors_params[0].max_speed = 0.0;
  EXPECT_FALSE(model.check_params(model_params));
  model_params.motors_params[0].min_speed = 0.0;
  model_params.motors_params[0].max_speed = 100.0;

  // Check motors time_constant
  model_params.motors_params[0].time_constant = -0.1;
  EXPECT_FALSE(model.check_params(model_params));
  model_params.motors_params[0].time_constant = 0.1;

  // Check motors rotational_inertia
  model_params.motors_params[0].rotational_inertia = -0.5;
  EXPECT_FALSE(model.check_params(model_params));
  model_params.motors_params[0].rotational_inertia = 0.5;

  // Check motors thrust_coefficient
  model_params.motors_params[0].thrust_coefficient = -2.0;
  EXPECT_FALSE(model.check_params(model_params));
  model_params.motors_params[0].thrust_coefficient = 2.0;

  // Check motors torque_coefficient
  model_params.motors_params[0].torque_coefficient = -1.0;
  EXPECT_FALSE(model.check_params(model_params));

  // Check update_params
  EXPECT_NO_THROW(model.update_params(model_params));
}

TEST(Model, getters) {
  const double thrust_coefficient = 2.0;
  const double torque_coefficient = 1.0;
  const double x_dist             = 0.2;
  const double y_dist             = 0.3;
  const double min_speed          = 0.0;
  const double max_speed          = 100.0;
  const double time_constant      = 0.1;
  const double rotational_inertia = 0.5;

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

  // Check model getters methods
  EXPECT_NO_THROW(model.get_params());
  EXPECT_NO_THROW(model.get_motors());
  EXPECT_NO_THROW(model.get_num_rotors());
  EXPECT_EQ(model.get_mass(), model_params.vehicle_mass);
  EXPECT_EQ(model.get_vehicle_inertia(), model_params.vehicle_inertia);
  EXPECT_EQ(model.get_vehicle_aero_moment_coefficient(),
            model_params.vehicle_aero_moment_coefficient);
  EXPECT_EQ(model.get_vehicle_drag_coefficient(), model_params.vehicle_drag_coefficient);
  EXPECT_EQ(model.get_gravity(), model_params.gravity);
  EXPECT_EQ(model.get_force_process_noise_auto_correlation(),
            model_params.force_process_noise_auto_correlation);
  EXPECT_EQ(model.get_moment_process_noise_auto_correlation(),
            model_params.moment_process_noise_auto_correlation);

  EXPECT_NO_THROW(model.get_stochastic_force());
  EXPECT_NO_THROW(model.get_stochastic_moment());
  EXPECT_NO_THROW(model.get_mixer_torque_matrix());
  EXPECT_NO_THROW(model.get_mixer_force_vector());
  EXPECT_NO_THROW(model.get_mixer_force_matrix());
  EXPECT_NO_THROW(model.get_mixer_inertia_matrix());
}

TEST(Model, public_methods) {
  const double thrust_coefficient = 2.0;
  const double torque_coefficient = 1.0;
  const double x_dist             = 0.2;
  const double y_dist             = 0.3;
  const double min_speed          = 0.0;
  const double max_speed          = 100.0;
  const double time_constant      = 0.1;
  const double rotational_inertia = 0.5;

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

  // Check model public methods
  EXPECT_NO_THROW(model.update_params(model_params));
  EXPECT_NO_THROW(model.set_stochastic_noise(0.1));
  EXPECT_NO_THROW(model.set_stochastic_noise(-0.1));
  EXPECT_NO_THROW(model.get_aerodynamic_moment(Eigen::Vector3d(1.0, 1.0, 1.0)));
  EXPECT_NO_THROW(model.get_drag_force(Eigen::Vector3d(1.0, 1.0, 1.0)));
  EXPECT_NO_THROW(model.get_force_thrust_by_motors(Eigen::Vector4d(1.0, 1.0, 1.0, 1.0)));
  EXPECT_NO_THROW(model.get_force_vector_thrust_by_motors(Eigen::Vector4d(1.0, 1.0, 1.0, 1.0)));
  EXPECT_NO_THROW(model.get_torque_by_motors(Eigen::Vector4d(1.0, 1.0, 1.0, 1.0),
                                             Eigen::Vector4d(1.0, 1.0, 1.0, 1.0)));
}

TEST(Model, static_methods_mixer_matrix) {
  // compute_mixer_matrix_4D
  // compute_mixer_matrix_6D

  // Mixer matrix params
  const double thrust_coefficient = 2.0;
  const double torque_coefficient = 1.0;
  const double x_dist             = 0.2;
  const double y_dist             = 0.3;
  const double radius             = 0.5;
  const double min_speed          = 0.0;
  const double max_speed          = 100.0;
  const double time_constant      = 0.1;
  const double rotational_inertia = 0.5;

  // Create quadrotor model
  std::vector<MotorParams<double>> motors;
  motors = Model<double, 4>::create_quadrotor_x_config(thrust_coefficient, torque_coefficient,
                                                       x_dist, y_dist, min_speed, max_speed,
                                                       time_constant, rotational_inertia);
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
  Model<double, 4> model_quad = Model<double, 4>(model_params);

  // Create hexacopter model
  std::vector<MotorParams<double>> motors_hexa;
  motors_hexa = Model<double, 6>::create_hexacopter_config(thrust_coefficient, torque_coefficient,
                                                           radius, min_speed, max_speed,
                                                           time_constant, rotational_inertia);
  ModelParams<double, 6> model_hexa_params;
  // Motor characteristics
  model_hexa_params.motors_params = motors;
  // Vehicle characteristics
  model_hexa_params.vehicle_mass    = 1.0;
  model_hexa_params.vehicle_inertia = Eigen::Vector3d(0.0049, 0.0049, 0.0069).asDiagonal();
  // Aerodynamic characteristics
  model_hexa_params.vehicle_aero_moment_coefficient =
      Eigen::Vector3d(0.003, 0.003, 0.003).asDiagonal();
  model_hexa_params.vehicle_drag_coefficient = 0.1;
  // Environment characteristics
  model_hexa_params.gravity                               = Eigen::Vector3d(0.0, 0.0, -9.81);
  model_hexa_params.force_process_noise_auto_correlation  = 5.00e-5;
  model_hexa_params.moment_process_noise_auto_correlation = 1.25e-7;
  model_hexa_params.motors_params                         = motors_hexa;
  Model<double, 6> model_hexa                             = Model<double, 6>(model_hexa_params);

  // Check compute mixer matrix for quadrotor
  EXPECT_NO_THROW(model_quad.compute_mixer_matrix<4>(motors));
  EXPECT_NO_THROW(model_quad.compute_mixer_matrix<6>(motors));
  Eigen::Matrix<double, 4, 4> mixing_matrix_4D_4rotors = model_quad.compute_mixer_matrix<4>(motors);
  Eigen::Matrix<double, 6, 4> mixing_matrix_6D_4rotors = model_quad.compute_mixer_matrix<6>(motors);

  // Check compute mixer matrix for hexacopter
  EXPECT_NO_THROW(model_hexa.compute_mixer_matrix<4>(motors));
  EXPECT_NO_THROW(model_hexa.compute_mixer_matrix<6>(motors));
  Eigen::Matrix<double, 4, 6> mixing_matrix_4D_6rotors = model_hexa.compute_mixer_matrix<4>(motors);
  Eigen::Matrix<double, 6, 6> mixing_matrix_6D_6rotors = model_hexa.compute_mixer_matrix<6>(motors);
}

TEST(Model, get_aerodynamic_moment) {
  ModelParams<double, 4> model_params = get_model_params();
  Model<double, 4> model              = Model<double, 4>(model_params);

  // Check aerodynamic moment
  Eigen::Vector3d vehicle_angular_velocity = Eigen::Vector3d(1.0, 1.0, 1.0);
  Eigen::Vector3d aerodynamic_moment       = model.get_aerodynamic_moment(vehicle_angular_velocity);

  // If coefficients are zero, the aerodynamic moment should be zero
  model_params.vehicle_aero_moment_coefficient = Eigen::Vector3d(0.0, 0.0, 0.0).asDiagonal();
  model.update_params(model_params);
  EXPECT_EQ(model.get_aerodynamic_moment(vehicle_angular_velocity), Eigen::Vector3d::Zero());

  // If coefficients are not zero, the aerodynamic moment should be different
  // from zero
  model_params.vehicle_aero_moment_coefficient = Eigen::Vector3d(1.0, 1.0, 1.0).asDiagonal();
  model.update_params(model_params);
  EXPECT_NE(model.get_aerodynamic_moment(vehicle_angular_velocity), Eigen::Vector3d::Zero());

  // If vehicle angular velocity is zero, the aerodynamic moment should be zero
  vehicle_angular_velocity = Eigen::Vector3d::Zero();
  EXPECT_EQ(model.get_aerodynamic_moment(vehicle_angular_velocity), Eigen::Vector3d::Zero());

  // The aerodynamic moment sign should be opposite to the vehicle angular
  // velocity
  vehicle_angular_velocity = Eigen::Vector3d(1.0, 1.0, 1.0);
  aerodynamic_moment       = model.get_aerodynamic_moment(vehicle_angular_velocity);
  EXPECT_LT(aerodynamic_moment(0), 0.0);
  EXPECT_LT(aerodynamic_moment(1), 0.0);
  EXPECT_LT(aerodynamic_moment(2), 0.0);

  vehicle_angular_velocity = Eigen::Vector3d(-1.0, -1.0, -1.0);
  aerodynamic_moment       = model.get_aerodynamic_moment(vehicle_angular_velocity);
  EXPECT_GT(aerodynamic_moment(0), 0.0);
  EXPECT_GT(aerodynamic_moment(1), 0.0);
  EXPECT_GT(aerodynamic_moment(2), 0.0);
}

TEST(Model, get_drag_force) {
  ModelParams<double, 4> model_params = get_model_params();
  Model<double, 4> model              = Model<double, 4>(model_params);

  // Check drag force
  Eigen::Vector3d vehicle_linear_velocity = Eigen::Vector3d(1.0, 1.0, 1.0);
  Eigen::Vector3d drag_force              = model.get_drag_force(vehicle_linear_velocity);

  // If coefficient is zero, the drag force should be zero
  model_params.vehicle_drag_coefficient = 0.0;
  model.update_params(model_params);
  EXPECT_EQ(model.get_drag_force(vehicle_linear_velocity), Eigen::Vector3d::Zero());

  // If coefficient is not zero, the drag force should be different from zero
  model_params.vehicle_drag_coefficient = 1.0;
  model.update_params(model_params);
  EXPECT_NE(model.get_drag_force(vehicle_linear_velocity), Eigen::Vector3d::Zero());

  // If vehicle linear velocity is zero, the drag force should be zero
  vehicle_linear_velocity = Eigen::Vector3d::Zero();
  EXPECT_EQ(model.get_drag_force(vehicle_linear_velocity), Eigen::Vector3d::Zero());

  // The drag force sign should be opposite to the vehicle linear velocity
  vehicle_linear_velocity = Eigen::Vector3d(1.0, 1.0, 1.0);
  drag_force              = model.get_drag_force(vehicle_linear_velocity);
  EXPECT_LT(drag_force(0), 0.0);
  EXPECT_LT(drag_force(1), 0.0);
  EXPECT_LT(drag_force(2), 0.0);

  vehicle_linear_velocity = Eigen::Vector3d(-1.0, -1.0, -1.0);
  drag_force              = model.get_drag_force(vehicle_linear_velocity);
  EXPECT_GT(drag_force(0), 0.0);
  EXPECT_GT(drag_force(1), 0.0);
  EXPECT_GT(drag_force(2), 0.0);
}

TEST(Model, get_force_thrust_by_motors) {
  ModelParams<double, 4> model_params     = get_model_params();
  Model<double, 4> model                  = Model<double, 4>(model_params);
  Eigen::Vector4d motors_angular_velocity = Eigen::Vector4d(1.0, 1.0, 1.0, 1.0);

  // Check thrust force
  double thrust_force = model.get_force_thrust_by_motors(motors_angular_velocity);
  Eigen::Vector3d thrust_force_vector =
      model.get_force_vector_thrust_by_motors(motors_angular_velocity);

  // If coefficient is zero, the thrust force should be zero
  model_params = get_model_params(0.0);
  model.update_params(model_params);
  EXPECT_EQ(model.get_force_thrust_by_motors(motors_angular_velocity), 0.0);
  EXPECT_EQ(model.get_force_vector_thrust_by_motors(motors_angular_velocity),
            Eigen::Vector3d::Zero());

  // If coefficient is not zero, the thrust force should be different from zero
  model_params = get_model_params(2.0);
  model.update_params(model_params);
  EXPECT_NE(model.get_force_thrust_by_motors(motors_angular_velocity), 0.0);
  EXPECT_NE(model.get_force_vector_thrust_by_motors(motors_angular_velocity),
            Eigen::Vector3d::Zero());

  // If motors angular velocity is zero, the thrust force should be zero
  motors_angular_velocity = Eigen::Vector4d::Zero();
  EXPECT_EQ(model.get_force_thrust_by_motors(motors_angular_velocity), 0.0);
  EXPECT_EQ(model.get_force_vector_thrust_by_motors(motors_angular_velocity),
            Eigen::Vector3d::Zero());

  // The thrust force sign should be equal to the motors angular velocity
  motors_angular_velocity = Eigen::Vector4d(1.0, 1.0, 1.0, 1.0);
  thrust_force            = model.get_force_thrust_by_motors(motors_angular_velocity);
  thrust_force_vector     = model.get_force_vector_thrust_by_motors(motors_angular_velocity);
  EXPECT_GT(thrust_force, 0.0);
  EXPECT_GT(thrust_force_vector(2), 0.0);
  EXPECT_EQ(thrust_force, model_params.motors_params[0].thrust_coefficient +
                              model_params.motors_params[1].thrust_coefficient +
                              model_params.motors_params[2].thrust_coefficient +
                              model_params.motors_params[3].thrust_coefficient);
}

TEST(Model, get_torque_by_motors) {
  const double thrust_coefficient = 2.0;
  const double torque_coefficient = 1.0;
  const double x_dist             = 0.2;
  const double y_dist             = 0.2;
  const double min_speed          = 0.0;
  const double max_speed          = 100.0;
  const double time_constant      = 0.1;
  const double rotational_inertia = 0.5;

  std::vector<MotorParams<double>> motors = Model<double, 4>::create_quadrotor_plus_config(
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

  Model<double, 4> model                      = Model<double, 4>(model_params);
  Eigen::Vector3d motor_torque                = Eigen::Vector3d::Zero();
  Eigen::Vector4d motors_angular_velocity     = Eigen::Vector4d(1.0, 1.0, 1.0, 1.0);
  Eigen::Vector4d motors_angular_acceleration = Eigen::Vector4d(1.0, 1.0, 1.0, 1.0);

  float low_speed  = 1000.0;
  float mid_speed  = 2000.0;
  float high_speed = 3000.0;

  // Move up
  motors_angular_velocity = Eigen::Vector4d(mid_speed, mid_speed, mid_speed, mid_speed);
  motor_torque = model.get_torque_by_motors(motors_angular_velocity, motors_angular_acceleration);
  EXPECT_EQ(motor_torque.x(), 0.0);
  EXPECT_EQ(motor_torque.y(), 0.0);
  EXPECT_EQ(motor_torque.z(), 0.0);

  // Move forward
  motors_angular_velocity = Eigen::Vector4d(low_speed, mid_speed, high_speed, mid_speed);
  motor_torque = model.get_torque_by_motors(motors_angular_velocity, motors_angular_acceleration);
  EXPECT_EQ(motor_torque.x(), 0.0);
  EXPECT_GT(motor_torque.y(), 0.0);
  EXPECT_GT(motor_torque.z(), 0.0);

  // Move backward
  motors_angular_velocity = Eigen::Vector4d(high_speed, mid_speed, low_speed, mid_speed);
  motor_torque = model.get_torque_by_motors(motors_angular_velocity, motors_angular_acceleration);
  EXPECT_EQ(motor_torque.x(), 0.0);
  EXPECT_LT(motor_torque.y(), 0.0);
  EXPECT_GT(motor_torque.z(), 0.0);

  // Move left
  motors_angular_velocity = Eigen::Vector4d(mid_speed, low_speed, mid_speed, high_speed);
  motor_torque = model.get_torque_by_motors(motors_angular_velocity, motors_angular_acceleration);
  EXPECT_LT(motor_torque.x(), 0.0);
  EXPECT_EQ(motor_torque.y(), 0.0);
  EXPECT_LT(motor_torque.z(), 0.0);

  // Move right
  motors_angular_velocity = Eigen::Vector4d(mid_speed, high_speed, mid_speed, low_speed);
  motor_torque = model.get_torque_by_motors(motors_angular_velocity, motors_angular_acceleration);
  EXPECT_GT(motor_torque.x(), 0.0);
  EXPECT_EQ(motor_torque.y(), 0.0);
  EXPECT_LT(motor_torque.z(), 0.0);

  // Rotate clockwise
  motors_angular_velocity = Eigen::Vector4d(low_speed, high_speed, low_speed, high_speed);
  motor_torque = model.get_torque_by_motors(motors_angular_velocity, motors_angular_acceleration);
  EXPECT_EQ(motor_torque.x(), 0.0);
  EXPECT_EQ(motor_torque.y(), 0.0);
  EXPECT_LT(motor_torque.z(), 0.0);

  // Rotate counter-clockwise
  motors_angular_velocity = Eigen::Vector4d(high_speed, low_speed, high_speed, low_speed);
  motor_torque = model.get_torque_by_motors(motors_angular_velocity, motors_angular_acceleration);
  EXPECT_EQ(motor_torque.x(), 0.0);
  EXPECT_EQ(motor_torque.y(), 0.0);
  EXPECT_GT(motor_torque.z(), 0.0);

  // Move diagonally forward left
  motors_angular_velocity = Eigen::Vector4d(low_speed, low_speed, high_speed, high_speed);
  motor_torque = model.get_torque_by_motors(motors_angular_velocity, motors_angular_acceleration);
  EXPECT_LT(motor_torque.x(), 0.0);
  EXPECT_EQ(-1.0 * motor_torque.x(), motor_torque.y());
  EXPECT_EQ(motor_torque.z(), 0.0);

  // Move diagonally forward right
  motors_angular_velocity = Eigen::Vector4d(low_speed, high_speed, high_speed, low_speed);
  motor_torque = model.get_torque_by_motors(motors_angular_velocity, motors_angular_acceleration);
  EXPECT_GT(motor_torque.x(), 0.0);
  EXPECT_EQ(motor_torque.x(), motor_torque.y());
  EXPECT_EQ(motor_torque.z(), 0.0);

  // Move diagonally backward left
  motors_angular_velocity = Eigen::Vector4d(high_speed, low_speed, low_speed, high_speed);
  motor_torque = model.get_torque_by_motors(motors_angular_velocity, motors_angular_acceleration);
  EXPECT_LT(motor_torque.x(), 0.0);
  EXPECT_EQ(motor_torque.x(), motor_torque.y());
  EXPECT_EQ(motor_torque.z(), 0.0);

  // Move diagonally backward right
  motors_angular_velocity = Eigen::Vector4d(high_speed, high_speed, low_speed, low_speed);
  motor_torque = model.get_torque_by_motors(motors_angular_velocity, motors_angular_acceleration);
  EXPECT_GT(motor_torque.x(), 0.0);
  EXPECT_EQ(motor_torque.x(), -1.0 * motor_torque.y());
  EXPECT_EQ(motor_torque.z(), 0.0);
}

}  // namespace multirotor::model

int main(int argc, char* argv[]) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
