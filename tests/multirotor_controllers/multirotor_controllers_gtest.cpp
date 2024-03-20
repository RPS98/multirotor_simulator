/*!*******************************************************************************************
 *  \file       multirotor_controller_gtest.cpp
 *  \brief      Class gtest
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

#include "multirotor_controllers/multirotor_controllers.hpp"

TEST(Controller, constructor) {
  EXPECT_NO_THROW(multirotor::controller::Controller multirotor_controller =
                      multirotor::controller::Controller());

  acro_controller::AcroControllerParams acro_controller_params =
      acro_controller::AcroControllerParams();
  indi_controller::IndiControllerParams indi_controller_params =
      indi_controller::IndiControllerParams();
  position_controller::PositionControllerParams position_controller_params =
      position_controller::PositionControllerParams();
  trajectory_controller::TrajectoryControllerParams trajectory_controller_params =
      trajectory_controller::TrajectoryControllerParams();
  velocity_controller::VelocityControllerParams velocity_controller_params =
      velocity_controller::VelocityControllerParams();
  EXPECT_NO_THROW(
      multirotor::controller::Controller multirotor_controller = multirotor::controller::Controller(
          acro_controller_params, indi_controller_params, position_controller_params,
          trajectory_controller_params, velocity_controller_params));
  multirotor::controller::ControllerParams controller_params;
  EXPECT_NO_THROW(multirotor::controller::Controller multirotor_controller =
                      multirotor::controller::Controller(controller_params));
}

TEST(Controller, public_methods) {
  acro_controller::AcroControllerParams acro_controller_params =
      acro_controller::AcroControllerParams();
  indi_controller::IndiControllerParams indi_controller_params =
      indi_controller::IndiControllerParams();
  position_controller::PositionControllerParams position_controller_params =
      position_controller::PositionControllerParams();
  trajectory_controller::TrajectoryControllerParams trajectory_controller_params =
      trajectory_controller::TrajectoryControllerParams();
  velocity_controller::VelocityControllerParams velocity_controller_params =
      velocity_controller::VelocityControllerParams();

  multirotor::controller::ControllerParams controller_params;
  controller_params.acro_controller_params       = acro_controller_params;
  controller_params.indi_controller_params       = indi_controller_params;
  controller_params.position_controller_params   = position_controller_params;
  controller_params.trajectory_controller_params = trajectory_controller_params;
  controller_params.velocity_controller_params   = velocity_controller_params;

  multirotor::controller::Controller multirotor_controller = multirotor::controller::Controller(
      acro_controller_params, indi_controller_params, position_controller_params,
      trajectory_controller_params, velocity_controller_params);

  // State
  Eigen::Vector3d current_position         = Eigen::Vector3d(0, 0, 0);
  Eigen::Vector3d current_velocity         = Eigen::Vector3d(0, 0, 0);
  Eigen::Matrix3d current_orientation      = Eigen::Matrix3d::Identity();
  Eigen::Quaterniond current_orientation_q = Eigen::Quaterniond::Identity();
  Eigen::Vector3d current_angular_velocity = Eigen::Vector3d(0, 0, 0);

  // Desired
  double thrust                            = 0;
  Eigen::Vector3d desired_angular_velocity = Eigen::Vector3d(0, 0, 0);

  Eigen::Vector3d desired_position     = Eigen::Vector3d(0, 0, 0);
  Eigen::Vector3d desired_velocity     = Eigen::Vector3d(0, 0, 0);
  Eigen::Vector3d desired_acceleration = Eigen::Vector3d(0, 0, 0);
  double desired_yaw                   = 0;

  // Time
  double dt = 0.01;

  // Output
  Eigen::Vector4d motors_angular_velocity = Eigen::Vector4d(0, 0, 0, 0);

  // Public methods
  EXPECT_NO_THROW(motors_angular_velocity =
                      multirotor_controller.convert_acro_to_motor_angular_velocity(
                          current_angular_velocity, thrust, desired_angular_velocity, dt));
  EXPECT_NO_THROW(std::tie(thrust, desired_angular_velocity) =
                      multirotor_controller.convert_linear_acceleration_to_acro(
                          desired_acceleration, current_orientation, desired_yaw, dt));
  EXPECT_NO_THROW(desired_acceleration =
                      multirotor_controller.convert_trajectory_to_linear_acceleration(
                          current_position, current_velocity, desired_position, desired_velocity,
                          desired_acceleration, dt));
  EXPECT_NO_THROW(desired_acceleration =
                      multirotor_controller.convert_linear_velocity_to_linear_acceleration(
                          current_velocity, desired_velocity, dt));
  EXPECT_NO_THROW(desired_velocity = multirotor_controller.convert_position_to_linear_velocity(
                      current_position, desired_position, dt));

  EXPECT_NO_THROW(motors_angular_velocity = multirotor_controller.compute_acro_control(
                      current_angular_velocity, thrust, desired_angular_velocity, dt));
  EXPECT_NO_THROW(
      motors_angular_velocity = multirotor_controller.compute_acceleration_control(
          current_orientation, current_angular_velocity, desired_acceleration, desired_yaw, dt));
  EXPECT_NO_THROW(motors_angular_velocity = multirotor_controller.compute_trajectory_control(
                      current_position, current_velocity, current_orientation,
                      current_angular_velocity, desired_position, desired_velocity,
                      desired_acceleration, desired_yaw, dt));
  EXPECT_NO_THROW(motors_angular_velocity = multirotor_controller.compute_velocity_control(
                      current_velocity, current_orientation, current_angular_velocity,
                      desired_velocity, desired_yaw, dt));
  EXPECT_NO_THROW(motors_angular_velocity = multirotor_controller.compute_position_control(
                      current_position, current_orientation, current_velocity,
                      current_angular_velocity, desired_position, desired_yaw, dt));

  EXPECT_NO_THROW(multirotor_controller.reset_controller());
  EXPECT_NO_THROW(multirotor_controller.update_params(controller_params));
  EXPECT_NO_THROW(multirotor_controller.update_acro_controller_params(acro_controller_params));
  EXPECT_NO_THROW(multirotor_controller.update_indi_controller_params(indi_controller_params));
  EXPECT_NO_THROW(
      multirotor_controller.update_position_controller_params(position_controller_params));
  EXPECT_NO_THROW(
      multirotor_controller.update_trajectory_controller_params(trajectory_controller_params));
  EXPECT_NO_THROW(
      multirotor_controller.update_velocity_controller_params(velocity_controller_params));
}

TEST(Controller, compute_acro_control) {
  multirotor::controller::ControllerParams controller_params;

  multirotor::controller::Controller multirotor_controller =
      multirotor::controller::Controller(controller_params);

  // State
  Eigen::Vector3d current_angular_velocity = Eigen::Vector3d(0, 0, 0);

  // Desired
  double thrust                            = 0;
  Eigen::Vector3d desired_angular_velocity = Eigen::Vector3d(0, 0, 0);

  // Time
  double dt = 0.01;

  // Output
  Eigen::Vector4d motors_angular_velocity = Eigen::Vector4d(0, 0, 0, 0);

  // Compute control
  motors_angular_velocity = multirotor_controller.compute_acro_control(
      current_angular_velocity, thrust, desired_angular_velocity, dt);
}

indi_controller::IndiControllerParams<double> get_indi_controller_params() {
  // INDI Controller Params
  indi_controller::IndiControllerParams indi_controller_params =
      indi_controller::IndiControllerParams();

  indi_controller_params.inertia = Eigen::Vector3d(0.0049, 0.0049, 0.0069).asDiagonal();
  // Quadrotor X configuration mixer matrix inverse
  indi_controller_params.mixer_matrix_inverse << 0.0, 0.0, 130890.0, -1.63613e+06, -1.63613e+06,
      961538.0, 0.0, 0.0, 130890.0, 1.63613e+06, -1.63613e+06, -961538.0, 0.0, 0.0, 130890.0,
      1.63613e+06, 1.63613e+06, 961538.0, 0.0, 0.0, 130890.0, -1.63613e+06, 1.63613e+06, -961538.0;

  indi_controller_params.pid_params.Kd_gains = Eigen::Vector3d::Ones() * 0.9;
  indi_controller_params.pid_params.Ki_gains = Eigen::Vector3d::Ones() * 3.0;
  indi_controller_params.pid_params.Kp_gains = Eigen::Vector3d::Ones() * 0.3;

  indi_controller_params.pid_params.antiwindup_cte      = Eigen::Vector3d::Ones() * 10.0;
  indi_controller_params.pid_params.alpha               = Eigen::Vector3d::Ones() * 0.1;
  indi_controller_params.pid_params.reset_integral_flag = false;

  indi_controller_params.pid_params.proportional_saturation_flag = true;
  indi_controller_params.pid_params.upper_output_saturation      = Eigen::Vector3d::Zero();
  indi_controller_params.pid_params.lower_output_saturation      = Eigen::Vector3d::Zero();
  return indi_controller_params;
}

acro_controller::AcroControllerParams<double> get_acro_controller_params() {
  // Acro Controller Params
  acro_controller::AcroControllerParams acro_controller_params =
      acro_controller::AcroControllerParams();

  acro_controller_params.vehicle_mass = 1;
  acro_controller_params.kp_rot       = Eigen::Matrix3d::Identity() * 1;
  return acro_controller_params;
}

TEST(Controller, compute_trayectory_control) {
  // Trajectory Controller Params
  trajectory_controller::TrajectoryControllerParams trajectory_controller_params =
      trajectory_controller::TrajectoryControllerParams();

  trajectory_controller_params.pid_params.Kp_gains = Eigen::Vector3d::Ones();

  // Multirotor Controller
  multirotor::controller::ControllerParams controller_params;
  controller_params.acro_controller_params       = get_acro_controller_params();
  controller_params.indi_controller_params       = get_indi_controller_params();
  controller_params.trajectory_controller_params = trajectory_controller_params;

  multirotor::controller::Controller multirotor_controller =
      multirotor::controller::Controller(controller_params);

  // State
  Eigen::Vector3d current_position         = Eigen::Vector3d(0, 0, 0);
  Eigen::Vector3d current_velocity         = Eigen::Vector3d(0, 0, 0);
  Eigen::Matrix3d current_orientation      = Eigen::Matrix3d::Identity();
  Eigen::Vector3d current_angular_velocity = Eigen::Vector3d(0, 0, 0);

  // Desired
  Eigen::Vector3d desired_position     = Eigen::Vector3d(0, 0, 0);
  Eigen::Vector3d desired_velocity     = Eigen::Vector3d(0, 0, 0);
  Eigen::Vector3d desired_acceleration = Eigen::Vector3d(0, 0, 0);
  double desired_yaw                   = 0;

  // Time
  double dt = 0.01;

  // Output
  Eigen::Vector4d output_motor_angular_velocity = Eigen::Vector4d(0, 0, 0, 0);

  // Compute control
  multirotor_controller.reset_controller();
  output_motor_angular_velocity = multirotor_controller.compute_trajectory_control(
      current_position, current_velocity, current_orientation, current_angular_velocity,
      desired_position, desired_velocity, desired_acceleration, desired_yaw, dt);

  // Expected output

  // Hover
  multirotor_controller.reset_controller();
  output_motor_angular_velocity = multirotor_controller.compute_trajectory_control(
      current_position, current_velocity, current_orientation, current_angular_velocity,
      desired_position, desired_velocity, desired_acceleration, desired_yaw, dt);
  EXPECT_EQ(output_motor_angular_velocity[0], output_motor_angular_velocity[1]);
  EXPECT_EQ(output_motor_angular_velocity[1], output_motor_angular_velocity[2]);
  EXPECT_EQ(output_motor_angular_velocity[2], output_motor_angular_velocity[3]);

  // Move Up
  double hover_motors_angular_velocity = output_motor_angular_velocity[0];

  desired_position(2) = 1.0;
  multirotor_controller.reset_controller();
  output_motor_angular_velocity = multirotor_controller.compute_trajectory_control(
      current_position, current_velocity, current_orientation, current_angular_velocity,
      desired_position, desired_velocity, desired_acceleration, desired_yaw, dt);
  EXPECT_EQ(output_motor_angular_velocity[0], output_motor_angular_velocity[1]);
  EXPECT_EQ(output_motor_angular_velocity[1], output_motor_angular_velocity[2]);
  EXPECT_EQ(output_motor_angular_velocity[2], output_motor_angular_velocity[3]);
  EXPECT_GT(output_motor_angular_velocity[0], hover_motors_angular_velocity);
  EXPECT_GT(output_motor_angular_velocity[1], hover_motors_angular_velocity);
  EXPECT_GT(output_motor_angular_velocity[2], hover_motors_angular_velocity);
  EXPECT_GT(output_motor_angular_velocity[3], hover_motors_angular_velocity);
  desired_position(2) = 0.0;

  // Move Down
  desired_position(2) = -1.0;
  multirotor_controller.reset_controller();
  output_motor_angular_velocity = multirotor_controller.compute_trajectory_control(
      current_position, current_velocity, current_orientation, current_angular_velocity,
      desired_position, desired_velocity, desired_acceleration, desired_yaw, dt);
  EXPECT_EQ(output_motor_angular_velocity[0], output_motor_angular_velocity[1]);
  EXPECT_EQ(output_motor_angular_velocity[1], output_motor_angular_velocity[2]);
  EXPECT_EQ(output_motor_angular_velocity[2], output_motor_angular_velocity[3]);
  EXPECT_LT(output_motor_angular_velocity[0], hover_motors_angular_velocity);
  EXPECT_LT(output_motor_angular_velocity[1], hover_motors_angular_velocity);
  EXPECT_LT(output_motor_angular_velocity[2], hover_motors_angular_velocity);
  EXPECT_LT(output_motor_angular_velocity[3], hover_motors_angular_velocity);
  desired_position(2) = 0.0;

  // Move clockwise
  desired_yaw = -0.2;
  multirotor_controller.reset_controller();
  output_motor_angular_velocity = multirotor_controller.compute_trajectory_control(
      current_position, current_velocity, current_orientation, current_angular_velocity,
      desired_position, desired_velocity, desired_acceleration, desired_yaw, dt);
  EXPECT_EQ(output_motor_angular_velocity[0], output_motor_angular_velocity[2]);
  EXPECT_EQ(output_motor_angular_velocity[1], output_motor_angular_velocity[3]);
  EXPECT_LT(output_motor_angular_velocity[0], output_motor_angular_velocity[1]);
  desired_yaw = 0.0;

  // Move counterclockwise
  desired_yaw = 0.2;
  multirotor_controller.reset_controller();
  output_motor_angular_velocity = multirotor_controller.compute_trajectory_control(
      current_position, current_velocity, current_orientation, current_angular_velocity,
      desired_position, desired_velocity, desired_acceleration, desired_yaw, dt);
  EXPECT_EQ(output_motor_angular_velocity[0], output_motor_angular_velocity[2]);
  EXPECT_EQ(output_motor_angular_velocity[1], output_motor_angular_velocity[3]);
  EXPECT_GT(output_motor_angular_velocity[0], output_motor_angular_velocity[1]);
  desired_yaw = 0.0;

  // Move forward
  desired_position(0) = 1.0;
  multirotor_controller.reset_controller();
  output_motor_angular_velocity = multirotor_controller.compute_trajectory_control(
      current_position, current_velocity, current_orientation, current_angular_velocity,
      desired_position, desired_velocity, desired_acceleration, desired_yaw, dt);
  EXPECT_EQ(output_motor_angular_velocity[0], output_motor_angular_velocity[1]);
  EXPECT_EQ(output_motor_angular_velocity[2], output_motor_angular_velocity[3]);
  EXPECT_LT(output_motor_angular_velocity[0], output_motor_angular_velocity[2]);
  desired_position(0) = 0.0;

  // Move backward
  desired_position(0) = -1.0;
  multirotor_controller.reset_controller();
  output_motor_angular_velocity = multirotor_controller.compute_trajectory_control(
      current_position, current_velocity, current_orientation, current_angular_velocity,
      desired_position, desired_velocity, desired_acceleration, desired_yaw, dt);
  EXPECT_EQ(output_motor_angular_velocity[0], output_motor_angular_velocity[1]);
  EXPECT_EQ(output_motor_angular_velocity[2], output_motor_angular_velocity[3]);
  EXPECT_GT(output_motor_angular_velocity[0], output_motor_angular_velocity[2]);
  desired_position(0) = 0.0;

  // Move left
  desired_position(1) = 1.0;
  multirotor_controller.reset_controller();
  output_motor_angular_velocity = multirotor_controller.compute_trajectory_control(
      current_position, current_velocity, current_orientation, current_angular_velocity,
      desired_position, desired_velocity, desired_acceleration, desired_yaw, dt);
  EXPECT_EQ(output_motor_angular_velocity[0], output_motor_angular_velocity[3]);
  EXPECT_EQ(output_motor_angular_velocity[1], output_motor_angular_velocity[2]);
  EXPECT_GT(output_motor_angular_velocity[0], output_motor_angular_velocity[1]);
  desired_position(1) = 0.0;

  // Move right
  desired_position(1) = -1.0;
  multirotor_controller.reset_controller();
  output_motor_angular_velocity = multirotor_controller.compute_trajectory_control(
      current_position, current_velocity, current_orientation, current_angular_velocity,
      desired_position, desired_velocity, desired_acceleration, desired_yaw, dt);
  EXPECT_EQ(output_motor_angular_velocity[0], output_motor_angular_velocity[3]);
  EXPECT_EQ(output_motor_angular_velocity[1], output_motor_angular_velocity[2]);
  EXPECT_LT(output_motor_angular_velocity[0], output_motor_angular_velocity[1]);
  desired_position(1) = 0.0;
}

TEST(Controller, compute_velocity_control) {
  // Velocity Controller Params
  velocity_controller::VelocityControllerParams velocity_controller_params =
      velocity_controller::VelocityControllerParams();

  velocity_controller_params.pid_params.Kp_gains = Eigen::Vector3d::Ones();

  // Multirotor Controller
  multirotor::controller::ControllerParams controller_params;
  controller_params.acro_controller_params     = get_acro_controller_params();
  controller_params.indi_controller_params     = get_indi_controller_params();
  controller_params.velocity_controller_params = velocity_controller_params;

  multirotor::controller::Controller multirotor_controller =
      multirotor::controller::Controller(controller_params);

  // State
  Eigen::Vector3d current_velocity         = Eigen::Vector3d(0, 0, 0);
  Eigen::Matrix3d current_orientation      = Eigen::Matrix3d::Identity();
  Eigen::Vector3d current_angular_velocity = Eigen::Vector3d(0, 0, 0);

  // Desired
  Eigen::Vector3d desired_velocity = Eigen::Vector3d(0, 0, 0);
  double desired_yaw               = 0;

  // Time
  double dt = 0.01;

  // Output
  Eigen::Vector4d output_motor_angular_velocity = Eigen::Vector4d(0, 0, 0, 0);

  // Compute control
  multirotor_controller.reset_controller();
  output_motor_angular_velocity = multirotor_controller.compute_velocity_control(
      current_velocity, current_orientation, current_angular_velocity, desired_velocity,
      desired_yaw, dt);

  // Expected output

  // Hover
  multirotor_controller.reset_controller();
  output_motor_angular_velocity = multirotor_controller.compute_velocity_control(
      current_velocity, current_orientation, current_angular_velocity, desired_velocity,
      desired_yaw, dt);
  EXPECT_EQ(output_motor_angular_velocity[0], output_motor_angular_velocity[1]);
  EXPECT_EQ(output_motor_angular_velocity[1], output_motor_angular_velocity[2]);
  EXPECT_EQ(output_motor_angular_velocity[2], output_motor_angular_velocity[3]);

  // Move Up
  double hover_motors_angular_velocity = output_motor_angular_velocity[0];

  desired_velocity(2) = 1.0;
  multirotor_controller.reset_controller();
  output_motor_angular_velocity = multirotor_controller.compute_velocity_control(
      current_velocity, current_orientation, current_angular_velocity, desired_velocity,
      desired_yaw, dt);
  EXPECT_EQ(output_motor_angular_velocity[0], output_motor_angular_velocity[1]);
  EXPECT_EQ(output_motor_angular_velocity[1], output_motor_angular_velocity[2]);
  EXPECT_EQ(output_motor_angular_velocity[2], output_motor_angular_velocity[3]);
  EXPECT_GT(output_motor_angular_velocity[0], hover_motors_angular_velocity);
  EXPECT_GT(output_motor_angular_velocity[1], hover_motors_angular_velocity);
  EXPECT_GT(output_motor_angular_velocity[2], hover_motors_angular_velocity);
  EXPECT_GT(output_motor_angular_velocity[3], hover_motors_angular_velocity);
  desired_velocity(2) = 0.0;

  // Move Down
  desired_velocity(2) = -1.0;
  multirotor_controller.reset_controller();
  output_motor_angular_velocity = multirotor_controller.compute_velocity_control(
      current_velocity, current_orientation, current_angular_velocity, desired_velocity,
      desired_yaw, dt);
  EXPECT_EQ(output_motor_angular_velocity[0], output_motor_angular_velocity[1]);
  EXPECT_EQ(output_motor_angular_velocity[1], output_motor_angular_velocity[2]);
  EXPECT_EQ(output_motor_angular_velocity[2], output_motor_angular_velocity[3]);
  EXPECT_LT(output_motor_angular_velocity[0], hover_motors_angular_velocity);
  EXPECT_LT(output_motor_angular_velocity[1], hover_motors_angular_velocity);
  EXPECT_LT(output_motor_angular_velocity[2], hover_motors_angular_velocity);
  EXPECT_LT(output_motor_angular_velocity[3], hover_motors_angular_velocity);
  desired_velocity(2) = 0.0;

  // Move clockwise
  desired_yaw = -0.2;
  multirotor_controller.reset_controller();
  output_motor_angular_velocity = multirotor_controller.compute_velocity_control(
      current_velocity, current_orientation, current_angular_velocity, desired_velocity,
      desired_yaw, dt);
  EXPECT_EQ(output_motor_angular_velocity[0], output_motor_angular_velocity[2]);
  EXPECT_EQ(output_motor_angular_velocity[1], output_motor_angular_velocity[3]);
  EXPECT_LT(output_motor_angular_velocity[0], output_motor_angular_velocity[1]);
  desired_yaw = 0.0;

  // Move counterclockwise
  desired_yaw = 0.2;
  multirotor_controller.reset_controller();
  output_motor_angular_velocity = multirotor_controller.compute_velocity_control(
      current_velocity, current_orientation, current_angular_velocity, desired_velocity,
      desired_yaw, dt);
  EXPECT_EQ(output_motor_angular_velocity[0], output_motor_angular_velocity[2]);
  EXPECT_EQ(output_motor_angular_velocity[1], output_motor_angular_velocity[3]);
  EXPECT_GT(output_motor_angular_velocity[0], output_motor_angular_velocity[1]);
  desired_yaw = 0.0;

  // Move forward
  desired_velocity(0) = 1.0;
  multirotor_controller.reset_controller();
  output_motor_angular_velocity = multirotor_controller.compute_velocity_control(
      current_velocity, current_orientation, current_angular_velocity, desired_velocity,
      desired_yaw, dt);
  EXPECT_EQ(output_motor_angular_velocity[0], output_motor_angular_velocity[1]);
  EXPECT_EQ(output_motor_angular_velocity[2], output_motor_angular_velocity[3]);
  EXPECT_LT(output_motor_angular_velocity[0], output_motor_angular_velocity[2]);
  desired_velocity(0) = 0.0;

  // Move backward
  desired_velocity(0) = -1.0;
  multirotor_controller.reset_controller();
  output_motor_angular_velocity = multirotor_controller.compute_velocity_control(
      current_velocity, current_orientation, current_angular_velocity, desired_velocity,
      desired_yaw, dt);
  EXPECT_EQ(output_motor_angular_velocity[0], output_motor_angular_velocity[1]);
  EXPECT_EQ(output_motor_angular_velocity[2], output_motor_angular_velocity[3]);
  EXPECT_GT(output_motor_angular_velocity[0], output_motor_angular_velocity[2]);
  desired_velocity(0) = 0.0;

  // Move left
  desired_velocity(1) = 1.0;
  multirotor_controller.reset_controller();
  output_motor_angular_velocity = multirotor_controller.compute_velocity_control(
      current_velocity, current_orientation, current_angular_velocity, desired_velocity,
      desired_yaw, dt);
  EXPECT_EQ(output_motor_angular_velocity[0], output_motor_angular_velocity[3]);
  EXPECT_EQ(output_motor_angular_velocity[1], output_motor_angular_velocity[2]);
  EXPECT_GT(output_motor_angular_velocity[0], output_motor_angular_velocity[1]);
  desired_velocity(1) = 0.0;

  // Move right
  desired_velocity(1) = -1.0;
  multirotor_controller.reset_controller();
  output_motor_angular_velocity = multirotor_controller.compute_velocity_control(
      current_velocity, current_orientation, current_angular_velocity, desired_velocity,
      desired_yaw, dt);
  EXPECT_EQ(output_motor_angular_velocity[0], output_motor_angular_velocity[3]);
  EXPECT_EQ(output_motor_angular_velocity[1], output_motor_angular_velocity[2]);
  EXPECT_LT(output_motor_angular_velocity[0], output_motor_angular_velocity[1]);
  desired_velocity(1) = 0.0;
}

TEST(Controller, compute_position_control) {
  // Velocity Controller Params
  velocity_controller::VelocityControllerParams velocity_controller_params =
      velocity_controller::VelocityControllerParams();

  velocity_controller_params.pid_params.Kp_gains = Eigen::Vector3d::Ones();

  // Position Controller Params
  position_controller::PositionControllerParams position_controller_params =
      position_controller::PositionControllerParams();

  position_controller_params.pid_params.Kp_gains = Eigen::Vector3d::Ones();

  // Multirotor Controller
  multirotor::controller::ControllerParams controller_params;
  controller_params.acro_controller_params     = get_acro_controller_params();
  controller_params.indi_controller_params     = get_indi_controller_params();
  controller_params.velocity_controller_params = velocity_controller_params;
  controller_params.position_controller_params = position_controller_params;

  multirotor::controller::Controller multirotor_controller =
      multirotor::controller::Controller(controller_params);

  // State
  Eigen::Vector3d current_position         = Eigen::Vector3d(0, 0, 0);
  Eigen::Vector3d current_velocity         = Eigen::Vector3d(0, 0, 0);
  Eigen::Matrix3d current_orientation      = Eigen::Matrix3d::Identity();
  Eigen::Vector3d current_angular_velocity = Eigen::Vector3d(0, 0, 0);

  // Desired
  Eigen::Vector3d desired_position = Eigen::Vector3d(0, 0, 0);
  double desired_yaw               = 0;

  // Time
  double dt = 0.01;

  // Output
  Eigen::Vector4d output_motor_angular_velocity = Eigen::Vector4d(0, 0, 0, 0);

  // Compute control
  multirotor_controller.reset_controller();
  output_motor_angular_velocity = multirotor_controller.compute_position_control(
      current_position, current_orientation, current_velocity, current_angular_velocity,
      desired_position, desired_yaw, dt);

  // Expected output

  // Hover
  multirotor_controller.reset_controller();
  output_motor_angular_velocity = multirotor_controller.compute_position_control(
      current_position, current_orientation, current_velocity, current_angular_velocity,
      desired_position, desired_yaw, dt);
  EXPECT_EQ(output_motor_angular_velocity[0], output_motor_angular_velocity[1]);
  EXPECT_EQ(output_motor_angular_velocity[1], output_motor_angular_velocity[2]);
  EXPECT_EQ(output_motor_angular_velocity[2], output_motor_angular_velocity[3]);

  // Move Up
  double hover_motors_angular_velocity = output_motor_angular_velocity[0];

  desired_position(2) = 1.0;
  multirotor_controller.reset_controller();
  output_motor_angular_velocity = multirotor_controller.compute_position_control(
      current_position, current_orientation, current_velocity, current_angular_velocity,
      desired_position, desired_yaw, dt);
  EXPECT_EQ(output_motor_angular_velocity[0], output_motor_angular_velocity[1]);
  EXPECT_EQ(output_motor_angular_velocity[1], output_motor_angular_velocity[2]);
  EXPECT_EQ(output_motor_angular_velocity[2], output_motor_angular_velocity[3]);
  EXPECT_GT(output_motor_angular_velocity[0], hover_motors_angular_velocity);
  EXPECT_GT(output_motor_angular_velocity[1], hover_motors_angular_velocity);
  EXPECT_GT(output_motor_angular_velocity[2], hover_motors_angular_velocity);
  EXPECT_GT(output_motor_angular_velocity[3], hover_motors_angular_velocity);
  desired_position(2) = 0.0;

  // Move Down
  desired_position(2) = -1.0;
  multirotor_controller.reset_controller();
  output_motor_angular_velocity = multirotor_controller.compute_position_control(
      current_position, current_orientation, current_velocity, current_angular_velocity,
      desired_position, desired_yaw, dt);
  EXPECT_EQ(output_motor_angular_velocity[0], output_motor_angular_velocity[1]);
  EXPECT_EQ(output_motor_angular_velocity[1], output_motor_angular_velocity[2]);
  EXPECT_EQ(output_motor_angular_velocity[2], output_motor_angular_velocity[3]);
  EXPECT_LT(output_motor_angular_velocity[0], hover_motors_angular_velocity);
  EXPECT_LT(output_motor_angular_velocity[1], hover_motors_angular_velocity);
  EXPECT_LT(output_motor_angular_velocity[2], hover_motors_angular_velocity);
  EXPECT_LT(output_motor_angular_velocity[3], hover_motors_angular_velocity);
  desired_position(2) = 0.0;

  // Move clockwise
  desired_yaw = -0.2;
  multirotor_controller.reset_controller();
  output_motor_angular_velocity = multirotor_controller.compute_position_control(
      current_position, current_orientation, current_velocity, current_angular_velocity,
      desired_position, desired_yaw, dt);
  EXPECT_EQ(output_motor_angular_velocity[0], output_motor_angular_velocity[2]);
  EXPECT_EQ(output_motor_angular_velocity[1], output_motor_angular_velocity[3]);
  EXPECT_LT(output_motor_angular_velocity[0], output_motor_angular_velocity[1]);
  desired_yaw = 0.0;

  // Move counterclockwise
  desired_yaw = 0.2;
  multirotor_controller.reset_controller();
  output_motor_angular_velocity = multirotor_controller.compute_position_control(
      current_position, current_orientation, current_velocity, current_angular_velocity,
      desired_position, desired_yaw, dt);
  EXPECT_EQ(output_motor_angular_velocity[0], output_motor_angular_velocity[2]);
  EXPECT_EQ(output_motor_angular_velocity[1], output_motor_angular_velocity[3]);
  EXPECT_GT(output_motor_angular_velocity[0], output_motor_angular_velocity[1]);
  desired_yaw = 0.0;

  // Move forward
  desired_position(0) = 1.0;
  multirotor_controller.reset_controller();
  output_motor_angular_velocity = multirotor_controller.compute_position_control(
      current_position, current_orientation, current_velocity, current_angular_velocity,
      desired_position, desired_yaw, dt);
  EXPECT_EQ(output_motor_angular_velocity[0], output_motor_angular_velocity[1]);
  EXPECT_EQ(output_motor_angular_velocity[2], output_motor_angular_velocity[3]);
  EXPECT_LT(output_motor_angular_velocity[0], output_motor_angular_velocity[2]);
  desired_position(0) = 0.0;

  // Move backward
  desired_position(0) = -1.0;
  multirotor_controller.reset_controller();
  output_motor_angular_velocity = multirotor_controller.compute_position_control(
      current_position, current_orientation, current_velocity, current_angular_velocity,
      desired_position, desired_yaw, dt);
  EXPECT_EQ(output_motor_angular_velocity[0], output_motor_angular_velocity[1]);
  EXPECT_EQ(output_motor_angular_velocity[2], output_motor_angular_velocity[3]);
  EXPECT_GT(output_motor_angular_velocity[0], output_motor_angular_velocity[2]);
  desired_position(0) = 0.0;

  // Move left
  desired_position(1) = 1.0;
  multirotor_controller.reset_controller();
  output_motor_angular_velocity = multirotor_controller.compute_position_control(
      current_position, current_orientation, current_velocity, current_angular_velocity,
      desired_position, desired_yaw, dt);
  EXPECT_EQ(output_motor_angular_velocity[0], output_motor_angular_velocity[3]);
  EXPECT_EQ(output_motor_angular_velocity[1], output_motor_angular_velocity[2]);
  EXPECT_GT(output_motor_angular_velocity[0], output_motor_angular_velocity[1]);
  desired_position(1) = 0.0;

  // Move right
  desired_position(1) = -1.0;
  multirotor_controller.reset_controller();
  output_motor_angular_velocity = multirotor_controller.compute_position_control(
      current_position, current_orientation, current_velocity, current_angular_velocity,
      desired_position, desired_yaw, dt);
  EXPECT_EQ(output_motor_angular_velocity[0], output_motor_angular_velocity[3]);
  EXPECT_EQ(output_motor_angular_velocity[1], output_motor_angular_velocity[2]);
  EXPECT_LT(output_motor_angular_velocity[0], output_motor_angular_velocity[1]);
  desired_position(1) = 0.0;
}

int main(int argc, char* argv[]) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
