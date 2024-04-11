/*!*******************************************************************************************
 *  \file       velocity_controller_gtest.hpp
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

#include "gtest/gtest.h"

#include "multirotor_controllers/controllers/velocity_controller.hpp"

namespace velocity_controller {

pid_controller::PIDParams<> get_pid_params() {
  pid_controller::PIDParams pid_params;
  pid_params.Kp_gains = Eigen::Vector3d::Ones() * 3.0;
  pid_params.Ki_gains = Eigen::Vector3d::Ones() * 0.3;
  pid_params.Kd_gains = Eigen::Vector3d::Ones() * 0.9;

  pid_params.antiwindup_cte      = Eigen::Vector3d::Ones() * 10.0;
  pid_params.alpha               = Eigen::Vector3d::Ones() * 0.1;
  pid_params.reset_integral_flag = false;

  pid_params.proportional_saturation_flag = true;
  pid_params.upper_output_saturation      = Eigen::Vector3d::Zero();
  pid_params.lower_output_saturation      = Eigen::Vector3d::Zero();
  return pid_params;
}

TEST(VelocityController, constructor) {
  EXPECT_NO_THROW(VelocityController velocity_controller = VelocityController());
  pid_controller::PIDParams pid_params = get_pid_params();
  EXPECT_NO_THROW(VelocityController velocity_controller = VelocityController(pid_params));
  VelocityControllerParams velocity_controller_params;
  EXPECT_NO_THROW(VelocityController velocity_controller =
                      VelocityController(velocity_controller_params));
}

TEST(VelocityController, public_methods) {
  // State
  Eigen::Vector3d current_linear_velocity = Eigen::Vector3d::Zero();

  // Reference
  Eigen::Vector3d desired_linear_velocity = Eigen::Vector3d::Zero();

  // Time
  double dt = 0.001;

  // Controller
  VelocityControllerParams velocity_controller_params;
  pid_controller::PIDParams pid_params   = get_pid_params();
  velocity_controller_params.pid_params  = pid_params;
  VelocityController velocity_controller = VelocityController(velocity_controller_params);

  EXPECT_NO_THROW(velocity_controller.linear_velocity_to_linear_acceleration(
      current_linear_velocity, desired_linear_velocity, dt));
  EXPECT_NO_THROW(velocity_controller.update_pid_params(pid_params));
  EXPECT_NO_THROW(velocity_controller.update_params(velocity_controller_params));
  EXPECT_NO_THROW(velocity_controller.reset_controller());

  // Getters
  EXPECT_NO_THROW(velocity_controller.get_desired_linear_acceleration());
  EXPECT_NO_THROW(velocity_controller.get_velocity_error());
}

TEST(VelocityController, position_to_linear_velocity) {
  // State
  Eigen::Vector3d current_linear_velocity = Eigen::Vector3d::Zero();

  // Reference
  Eigen::Vector3d desired_linear_velocity = Eigen::Vector3d::Zero();

  // Output
  Eigen::Vector3d desired_linear_acceleration = Eigen::Vector3d::Zero();

  // Time
  double dt = 0.001;

  // Controller
  VelocityControllerParams velocity_controller_params;
  pid_controller::PIDParams pid_params   = get_pid_params();
  velocity_controller_params.pid_params  = pid_params;
  VelocityController velocity_controller = VelocityController(velocity_controller_params);

  // Hover
  velocity_controller.reset_controller();
  desired_linear_acceleration = velocity_controller.linear_velocity_to_linear_acceleration(
      current_linear_velocity, desired_linear_velocity, dt);
  EXPECT_EQ(desired_linear_acceleration.x(), 0.0);
  EXPECT_EQ(desired_linear_acceleration.y(), 0.0);
  EXPECT_EQ(desired_linear_acceleration.z(), 0.0);

  // Move up
  desired_linear_velocity = Eigen::Vector3d(0.0, 0.0, 1.0);
  velocity_controller.reset_controller();
  desired_linear_acceleration = velocity_controller.linear_velocity_to_linear_acceleration(
      current_linear_velocity, desired_linear_velocity, dt);
  EXPECT_EQ(desired_linear_acceleration.x(), 0.0);
  EXPECT_EQ(desired_linear_acceleration.y(), 0.0);
  EXPECT_GT(desired_linear_acceleration.z(), 0.0);

  // Move down
  desired_linear_velocity = Eigen::Vector3d(0.0, 0.0, -1.0);
  velocity_controller.reset_controller();
  desired_linear_acceleration = velocity_controller.linear_velocity_to_linear_acceleration(
      current_linear_velocity, desired_linear_velocity, dt);
  EXPECT_EQ(desired_linear_acceleration.x(), 0.0);
  EXPECT_EQ(desired_linear_acceleration.y(), 0.0);
  EXPECT_LT(desired_linear_acceleration.z(), 0.0);

  // Move forward
  desired_linear_velocity = Eigen::Vector3d(1.0, 0.0, 0.0);
  velocity_controller.reset_controller();
  desired_linear_acceleration = velocity_controller.linear_velocity_to_linear_acceleration(
      current_linear_velocity, desired_linear_velocity, dt);
  EXPECT_GT(desired_linear_acceleration.x(), 0.0);
  EXPECT_EQ(desired_linear_acceleration.y(), 0.0);
  EXPECT_EQ(desired_linear_acceleration.z(), 0.0);

  // Move backward
  desired_linear_velocity = Eigen::Vector3d(-1.0, 0.0, 0.0);
  velocity_controller.reset_controller();
  desired_linear_acceleration = velocity_controller.linear_velocity_to_linear_acceleration(
      current_linear_velocity, desired_linear_velocity, dt);
  EXPECT_LT(desired_linear_acceleration.x(), 0.0);
  EXPECT_EQ(desired_linear_acceleration.y(), 0.0);
  EXPECT_EQ(desired_linear_acceleration.z(), 0.0);

  // Move left
  desired_linear_velocity = Eigen::Vector3d(0.0, 1.0, 0.0);
  velocity_controller.reset_controller();
  desired_linear_acceleration = velocity_controller.linear_velocity_to_linear_acceleration(
      current_linear_velocity, desired_linear_velocity, dt);
  EXPECT_EQ(desired_linear_acceleration.x(), 0.0);
  EXPECT_GT(desired_linear_acceleration.y(), 0.0);
  EXPECT_EQ(desired_linear_acceleration.z(), 0.0);

  // Move right
  desired_linear_velocity = Eigen::Vector3d(0.0, -1.0, 0.0);
  velocity_controller.reset_controller();
  desired_linear_acceleration = velocity_controller.linear_velocity_to_linear_acceleration(
      current_linear_velocity, desired_linear_velocity, dt);
  EXPECT_EQ(desired_linear_acceleration.x(), 0.0);
  EXPECT_LT(desired_linear_acceleration.y(), 0.0);
  EXPECT_EQ(desired_linear_acceleration.z(), 0.0);

  // Move diagonally forward left
  desired_linear_velocity = Eigen::Vector3d(1.0, 1.0, 0.0);
  velocity_controller.reset_controller();
  desired_linear_acceleration = velocity_controller.linear_velocity_to_linear_acceleration(
      current_linear_velocity, desired_linear_velocity, dt);
  EXPECT_GT(desired_linear_acceleration.x(), 0.0);
  EXPECT_GT(desired_linear_acceleration.y(), 0.0);
  EXPECT_EQ(desired_linear_acceleration.z(), 0.0);

  // Move diagonally forward right
  desired_linear_velocity = Eigen::Vector3d(1.0, -1.0, 0.0);
  velocity_controller.reset_controller();
  desired_linear_acceleration = velocity_controller.linear_velocity_to_linear_acceleration(
      current_linear_velocity, desired_linear_velocity, dt);
  EXPECT_GT(desired_linear_acceleration.x(), 0.0);
  EXPECT_LT(desired_linear_acceleration.y(), 0.0);
  EXPECT_EQ(desired_linear_acceleration.z(), 0.0);

  // Move diagonally backward left
  desired_linear_velocity = Eigen::Vector3d(-1.0, 1.0, 0.0);
  velocity_controller.reset_controller();
  desired_linear_acceleration = velocity_controller.linear_velocity_to_linear_acceleration(
      current_linear_velocity, desired_linear_velocity, dt);
  EXPECT_LT(desired_linear_acceleration.x(), 0.0);
  EXPECT_GT(desired_linear_acceleration.y(), 0.0);
  EXPECT_EQ(desired_linear_acceleration.z(), 0.0);

  // Move diagonally backward right
  desired_linear_velocity = Eigen::Vector3d(-1.0, -1.0, 0.0);
  velocity_controller.reset_controller();
  desired_linear_acceleration = velocity_controller.linear_velocity_to_linear_acceleration(
      current_linear_velocity, desired_linear_velocity, dt);
  EXPECT_LT(desired_linear_acceleration.x(), 0.0);
  EXPECT_LT(desired_linear_acceleration.y(), 0.0);
  EXPECT_EQ(desired_linear_acceleration.z(), 0.0);
}

}  // namespace velocity_controller

int main(int argc, char *argv[]) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
