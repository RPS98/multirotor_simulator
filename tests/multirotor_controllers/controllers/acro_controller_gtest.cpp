/*!*******************************************************************************************
 *  \file       acro_controller_gtest.hpp
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

#include "multirotor_controllers/controllers/acro_controller.hpp"

namespace acro_controller {

TEST(AcroController, constructor) {
  EXPECT_NO_THROW(acro_controller::AcroController acro_controller =
                      acro_controller::AcroController());

  double vehcile_mass    = 1.0;
  Eigen::Matrix3d kp_rot = Eigen::Matrix3d::Identity();
  EXPECT_NO_THROW(acro_controller::AcroController acro_controller =
                      acro_controller::AcroController(vehcile_mass, kp_rot));

  acro_controller::AcroControllerParams params;
  EXPECT_NO_THROW(acro_controller::AcroController acro_controller =
                      acro_controller::AcroController(params));
}

TEST(AcroController, public_methods) {
  // State
  Eigen::Matrix3d current_orientation = Eigen::Matrix3d::Identity();

  // Reference
  double desired_yaw                   = 0.0;
  Eigen::Vector3d desired_acceleration = Eigen::Vector3d::Ones();

  // Output
  Eigen::Vector3d desired_thrust = Eigen::Vector3d::Ones();

  // Controller
  AcroControllerParams acro_controller_params;
  acro_controller_params.vehicle_mass = 1.0;
  acro_controller_params.kp_rot       = Eigen::Matrix3d::Identity();
  AcroController acro_controller      = AcroController(acro_controller_params);

  EXPECT_NO_THROW(acro_controller.acceleration_to_thrust(desired_acceleration));
  EXPECT_NO_THROW(acro_controller.yaw_angle_to_angular_velocity(current_orientation, desired_yaw,
                                                                desired_thrust));
  EXPECT_NO_THROW(acro_controller.update_params(acro_controller_params));
  EXPECT_NO_THROW(acro_controller.update_vehicle_mass(acro_controller_params.vehicle_mass));
  Eigen::Vector3d gravity = Eigen::Vector3d(0.0, 0.0, -9.81);
  EXPECT_NO_THROW(acro_controller.update_gravity(gravity));
  EXPECT_NO_THROW(acro_controller.update_kp_rot(acro_controller_params.kp_rot));

  // Getters
  EXPECT_NO_THROW(acro_controller.get_desired_angular_velocity());
  EXPECT_NO_THROW(acro_controller.get_desired_thrust());
  EXPECT_NO_THROW(acro_controller.get_vehicle_mass());
  EXPECT_NO_THROW(acro_controller.get_gravity());
  EXPECT_NO_THROW(acro_controller.get_kp_rot());
}

TEST(AcroController, control) {
  // State
  Eigen::Matrix3d current_orientation = Eigen::Matrix3d::Identity();

  // Reference
  double desired_yaw                   = 0.0;
  Eigen::Vector3d desired_acceleration = Eigen::Vector3d::Zero();

  // Output
  Eigen::Vector3d desired_thrust           = Eigen::Vector3d::Zero();
  Eigen::Vector3d desired_angular_velocity = Eigen::Vector3d::Zero();

  // Controller
  AcroControllerParams acro_controller_params;
  acro_controller_params.vehicle_mass = 1.0;
  acro_controller_params.kp_rot       = Eigen::Matrix3d::Identity();
  AcroController acro_controller      = AcroController(acro_controller_params);

  // Hover
  desired_acceleration = Eigen::Vector3d::Zero();
  desired_thrust       = acro_controller.acceleration_to_thrust(desired_acceleration);
  EXPECT_EQ(desired_thrust, Eigen::Vector3d(0.0, 0.0, acro_controller_params.vehicle_mass * 9.81));
  desired_angular_velocity = acro_controller.yaw_angle_to_angular_velocity(
      current_orientation, desired_yaw, desired_thrust);
  EXPECT_EQ(desired_angular_velocity.x(), 0.0);
  EXPECT_EQ(desired_angular_velocity.y(), 0.0);
  EXPECT_EQ(desired_angular_velocity.z(), 0.0);

  // Rotate clockwise
  desired_acceleration = Eigen::Vector3d(0.0, 0.0, 0.0);
  desired_thrust       = acro_controller.acceleration_to_thrust(desired_acceleration);
  EXPECT_EQ(desired_thrust.x(), 0.0);
  EXPECT_EQ(desired_thrust.y(), 0.0);
  EXPECT_EQ(desired_thrust.z(), acro_controller_params.vehicle_mass * 9.81);
  desired_yaw              = 0.1;
  desired_angular_velocity = acro_controller.yaw_angle_to_angular_velocity(
      current_orientation, desired_yaw, desired_thrust);
  EXPECT_EQ(desired_angular_velocity.x(), 0.0);
  EXPECT_EQ(desired_angular_velocity.y(), 0.0);
  EXPECT_GT(desired_angular_velocity.z(), 0.0);

  // Rotate counterclockwise
  desired_acceleration = Eigen::Vector3d(0.0, 0.0, 0.0);
  desired_thrust       = acro_controller.acceleration_to_thrust(desired_acceleration);
  EXPECT_EQ(desired_thrust.x(), 0.0);
  EXPECT_EQ(desired_thrust.y(), 0.0);
  EXPECT_EQ(desired_thrust.z(), acro_controller_params.vehicle_mass * 9.81);
  desired_yaw              = -0.1;
  desired_angular_velocity = acro_controller.yaw_angle_to_angular_velocity(
      current_orientation, desired_yaw, desired_thrust);
  EXPECT_EQ(desired_angular_velocity.x(), 0.0);
  EXPECT_EQ(desired_angular_velocity.y(), 0.0);
  EXPECT_LT(desired_angular_velocity.z(), 0.0);
  desired_yaw = 0.0;

  // Move up
  desired_acceleration = Eigen::Vector3d(0.0, 0.0, 1.0);
  desired_thrust       = acro_controller.acceleration_to_thrust(desired_acceleration);
  EXPECT_EQ(desired_thrust.x(), 0.0);
  EXPECT_EQ(desired_thrust.y(), 0.0);
  EXPECT_GT(desired_thrust.z(), acro_controller_params.vehicle_mass * 9.81);
  desired_angular_velocity = acro_controller.yaw_angle_to_angular_velocity(
      current_orientation, desired_yaw, desired_thrust);
  EXPECT_EQ(desired_angular_velocity.x(), 0.0);
  EXPECT_EQ(desired_angular_velocity.y(), 0.0);
  EXPECT_EQ(desired_angular_velocity.z(), 0.0);

  // Move down
  desired_acceleration = Eigen::Vector3d(0.0, 0.0, -1.0);
  desired_thrust       = acro_controller.acceleration_to_thrust(desired_acceleration);
  EXPECT_EQ(desired_thrust.x(), 0.0);
  EXPECT_EQ(desired_thrust.y(), 0.0);
  EXPECT_LT(desired_thrust.z(), acro_controller_params.vehicle_mass * 9.81);
  desired_angular_velocity = acro_controller.yaw_angle_to_angular_velocity(
      current_orientation, desired_yaw, desired_thrust);
  EXPECT_EQ(desired_angular_velocity.x(), 0.0);
  EXPECT_EQ(desired_angular_velocity.y(), 0.0);
  EXPECT_EQ(desired_angular_velocity.z(), 0.0);

  // Move forward
  desired_acceleration = Eigen::Vector3d(1.0, 0.0, 0.0);
  desired_thrust       = acro_controller.acceleration_to_thrust(desired_acceleration);
  EXPECT_GT(desired_thrust.x(), 0.0);
  EXPECT_EQ(desired_thrust.y(), 0.0);
  EXPECT_EQ(desired_thrust.z(), acro_controller_params.vehicle_mass * 9.81);
  desired_angular_velocity = acro_controller.yaw_angle_to_angular_velocity(
      current_orientation, desired_yaw, desired_thrust);
  EXPECT_EQ(desired_angular_velocity.x(), 0.0);
  EXPECT_GT(desired_angular_velocity.y(), 0.0);
  EXPECT_EQ(desired_angular_velocity.z(), 0.0);

  // Move backward
  desired_acceleration = Eigen::Vector3d(-1.0, 0.0, 0.0);
  desired_thrust       = acro_controller.acceleration_to_thrust(desired_acceleration);
  EXPECT_LT(desired_thrust.x(), 0.0);
  EXPECT_EQ(desired_thrust.y(), 0.0);
  EXPECT_EQ(desired_thrust.z(), acro_controller_params.vehicle_mass * 9.81);
  desired_angular_velocity = acro_controller.yaw_angle_to_angular_velocity(
      current_orientation, desired_yaw, desired_thrust);
  EXPECT_EQ(desired_angular_velocity.x(), 0.0);
  EXPECT_LT(desired_angular_velocity.y(), 0.0);
  EXPECT_EQ(desired_angular_velocity.z(), 0.0);

  // Move left
  desired_acceleration = Eigen::Vector3d(0.0, -1.0, 0.0);
  desired_thrust       = acro_controller.acceleration_to_thrust(desired_acceleration);
  EXPECT_EQ(desired_thrust.x(), 0.0);
  EXPECT_LT(desired_thrust.y(), 0.0);
  EXPECT_EQ(desired_thrust.z(), acro_controller_params.vehicle_mass * 9.81);
  desired_angular_velocity = acro_controller.yaw_angle_to_angular_velocity(
      current_orientation, desired_yaw, desired_thrust);
  EXPECT_GT(desired_angular_velocity.x(), 0.0);
  EXPECT_EQ(desired_angular_velocity.y(), 0.0);
  EXPECT_EQ(desired_angular_velocity.z(), 0.0);

  // Move right
  desired_acceleration = Eigen::Vector3d(0.0, 1.0, 0.0);
  desired_thrust       = acro_controller.acceleration_to_thrust(desired_acceleration);
  EXPECT_EQ(desired_thrust.x(), 0.0);
  EXPECT_GT(desired_thrust.y(), 0.0);
  EXPECT_EQ(desired_thrust.z(), acro_controller_params.vehicle_mass * 9.81);
  desired_angular_velocity = acro_controller.yaw_angle_to_angular_velocity(
      current_orientation, desired_yaw, desired_thrust);
  EXPECT_LT(desired_angular_velocity.x(), 0.0);
  EXPECT_EQ(desired_angular_velocity.y(), 0.0);
  EXPECT_EQ(desired_angular_velocity.z(), 0.0);

  // Move diagonally forward left
  desired_acceleration = Eigen::Vector3d(1.0, 1.0, 0.0);
  desired_thrust       = acro_controller.acceleration_to_thrust(desired_acceleration);
  EXPECT_GT(desired_thrust.x(), 0.0);
  EXPECT_GT(desired_thrust.y(), 0.0);
  EXPECT_EQ(desired_thrust.z(), acro_controller_params.vehicle_mass * 9.81);
  desired_angular_velocity = acro_controller.yaw_angle_to_angular_velocity(
      current_orientation, desired_yaw, desired_thrust);
  EXPECT_LT(desired_angular_velocity.x(), 0.0);
  EXPECT_GT(desired_angular_velocity.y(), 0.0);

  // Move diagonally forward right
  desired_acceleration = Eigen::Vector3d(1.0, -1.0, 0.0);
  desired_thrust       = acro_controller.acceleration_to_thrust(desired_acceleration);
  EXPECT_GT(desired_thrust.x(), 0.0);
  EXPECT_LT(desired_thrust.y(), 0.0);
  EXPECT_EQ(desired_thrust.z(), acro_controller_params.vehicle_mass * 9.81);
  desired_angular_velocity = acro_controller.yaw_angle_to_angular_velocity(
      current_orientation, desired_yaw, desired_thrust);
  EXPECT_GT(desired_angular_velocity.x(), 0.0);
  EXPECT_GT(desired_angular_velocity.y(), 0.0);

  // Move diagonally backward left
  desired_acceleration = Eigen::Vector3d(-1.0, 1.0, 0.0);
  desired_thrust       = acro_controller.acceleration_to_thrust(desired_acceleration);
  EXPECT_LT(desired_thrust.x(), 0.0);
  EXPECT_GT(desired_thrust.y(), 0.0);
  EXPECT_EQ(desired_thrust.z(), acro_controller_params.vehicle_mass * 9.81);
  desired_angular_velocity = acro_controller.yaw_angle_to_angular_velocity(
      current_orientation, desired_yaw, desired_thrust);
  EXPECT_LT(desired_angular_velocity.x(), 0.0);
  EXPECT_LT(desired_angular_velocity.y(), 0.0);

  // Move diagonally backward right
  desired_acceleration = Eigen::Vector3d(-1.0, -1.0, 0.0);
  desired_thrust       = acro_controller.acceleration_to_thrust(desired_acceleration);
  EXPECT_LT(desired_thrust.x(), 0.0);
  EXPECT_LT(desired_thrust.y(), 0.0);
  EXPECT_EQ(desired_thrust.z(), acro_controller_params.vehicle_mass * 9.81);
  desired_angular_velocity = acro_controller.yaw_angle_to_angular_velocity(
      current_orientation, desired_yaw, desired_thrust);
  EXPECT_GT(desired_angular_velocity.x(), 0.0);
  EXPECT_LT(desired_angular_velocity.y(), 0.0);
}

}  // namespace acro_controller

int main(int argc, char *argv[]) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
