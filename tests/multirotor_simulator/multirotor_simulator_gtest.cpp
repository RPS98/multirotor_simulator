/*!*******************************************************************************************
 *  \file       multirotor_simulator_gtest.cpp
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

#include "multirotor_simulator.hpp"

namespace multirotor {

TEST(Simulator, constructor) {
  EXPECT_NO_THROW(Simulator simulator = Simulator());
  SimulatorParams simulator_params;
  EXPECT_NO_THROW(Simulator simulator = Simulator(simulator_params));
}

TEST(Simulator, update) {
  SimulatorParams simulator_params;
  Simulator simulator = Simulator(simulator_params);

  double dt = 0.001;
  EXPECT_NO_THROW(simulator.update_dynamics(dt));
  EXPECT_NO_THROW(simulator.update_controller(dt));
  EXPECT_NO_THROW(simulator.update_imu(dt));
  EXPECT_NO_THROW(simulator.update_inertial_odometry(dt));
}

TEST(Simulator, control_simulator) {
  SimulatorParams simulator_params;
  Simulator simulator = Simulator(simulator_params);

  // EXPECT_NO_THROW(simulator.arm());
  EXPECT_NO_THROW(simulator.disarm());
  EXPECT_NO_THROW(simulator.enable_floor_collision());
  EXPECT_NO_THROW(simulator.enable_floor_collision(1.0));
  EXPECT_NO_THROW(simulator.disable_floor_collision());
}

TEST(Simulator, setters_references) {
  SimulatorParams simulator_params;
  Simulator simulator = Simulator(simulator_params);

  EXPECT_NO_THROW(simulator.set_control_mode(ControlMode::ACRO));
  Eigen::Vector4d reference_motors_angular_velocity = Eigen::Vector4d::Zero();
  EXPECT_NO_THROW(simulator.set_refence_motors_angular_velocity(reference_motors_angular_velocity));
  double reference_acro_thrust                    = 0.0;
  Eigen::Vector3d reference_acro_angular_velocity = Eigen::Vector3d::Zero();
  EXPECT_NO_THROW(
      simulator.set_reference_acro(reference_acro_thrust, reference_acro_angular_velocity));
  Eigen::Vector3d reference_position     = Eigen::Vector3d::Zero();
  Eigen::Vector3d reference_velocity     = Eigen::Vector3d::Zero();
  Eigen::Vector3d reference_acceleration = Eigen::Vector3d::Zero();
  double reference_yaw                   = 0.0;
  EXPECT_NO_THROW(simulator.set_reference_trajectory(reference_position, reference_velocity,
                                                     reference_acceleration, reference_yaw));
}

TEST(Simulator, getters) {
  SimulatorParams simulator_params;
  Simulator simulator = Simulator(simulator_params);

  // Getters
  dynamics::Dynamics dynamics = simulator.get_dynamics();
  EXPECT_NO_THROW(simulator.get_dynamics(dynamics));
  EXPECT_NO_THROW(dynamics::Dynamics dynamics_ = simulator.get_dynamics());
  EXPECT_NO_THROW(const dynamics::Dynamics dynamics_const = simulator.get_dynamics_const());

  controller::Controller controller = simulator.get_controller();
  EXPECT_NO_THROW(simulator.get_controller(controller));
  EXPECT_NO_THROW(controller::Controller controller_ = simulator.get_controller());
  EXPECT_NO_THROW(const controller::Controller controller_const = simulator.get_controller_const());

  imu::IMU imu = simulator.get_imu();
  EXPECT_NO_THROW(simulator.get_imu(imu));
  EXPECT_NO_THROW(imu::IMU imu_ = simulator.get_imu());
  EXPECT_NO_THROW(const imu::IMU imu_const = simulator.get_imu_const());

  imu::InertialOdometry inertial_odometry = simulator.get_inertial_odometry();
  EXPECT_NO_THROW(simulator.get_inertial_odometry(inertial_odometry));
  EXPECT_NO_THROW(imu::InertialOdometry inertial_odometry_ = simulator.get_inertial_odometry());
  EXPECT_NO_THROW(const imu::InertialOdometry inertial_odometry_const =
                      simulator.get_inertial_odometry_const());
}

TEST(Simulator, setters) {
  SimulatorParams simulator_params;
  Simulator simulator = Simulator(simulator_params);

  // Setters
  dynamics::DynamicsParams dynamics_params;
  EXPECT_NO_THROW(simulator.set_dynamics_params(dynamics_params));

  controller::ControllerParams controller_params;
  EXPECT_NO_THROW(simulator.set_controller_params(controller_params));

  imu::IMUParams imu_params;
  EXPECT_NO_THROW(simulator.set_imu_params(imu_params));

  imu::InertialOdometryParams inertial_odometry_params;
  EXPECT_NO_THROW(simulator.set_inertial_odometry_params(inertial_odometry_params));
}

}  // namespace multirotor

int main(int argc, char* argv[]) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
