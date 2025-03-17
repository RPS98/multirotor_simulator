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

SimulatorParams<double, 4> get_simulation_params_from_code() {
  // Initialize simulator params
  SimulatorParams p;

  // Initial state
  p.dynamics_params.state.kinematics.position             = Eigen::Vector3d(0.0, 0.0, 0.0);
  p.dynamics_params.state.kinematics.linear_velocity      = Eigen::Vector3d(0.0, 0.0, 0.0);
  p.dynamics_params.state.kinematics.linear_acceleration  = Eigen::Vector3d(0.0, 0.0, 0.0);
  p.dynamics_params.state.kinematics.orientation          = Eigen::Quaterniond(1, 0, 0, 0);
  p.dynamics_params.state.kinematics.angular_velocity     = Eigen::Vector3d(0.0, 0.0, 0.0);
  p.dynamics_params.state.kinematics.angular_acceleration = Eigen::Vector3d(0.0, 0.0, 0.0);

  // Vehicle parameters
  p.dynamics_params.model_params.gravity      = Eigen::Vector3d(0.0, 0.0, -9.81);
  p.dynamics_params.model_params.vehicle_mass = 1.0;
  p.dynamics_params.model_params.vehicle_inertia =
      Eigen::Vector3d(0.0049, 0.0049, 0.0069).asDiagonal();
  p.dynamics_params.model_params.vehicle_drag_coefficient = 0.1;
  p.dynamics_params.model_params.vehicle_aero_moment_coefficient =
      Eigen::Vector3d(0.003, 0.003, 0.003).asDiagonal();
  p.dynamics_params.model_params.force_process_noise_auto_correlation  = 0.0005;
  p.dynamics_params.model_params.moment_process_noise_auto_correlation = 1.25e-7;

  // Motors parameters
  const double thrust_coefficient = 1.91e-6;
  const double torque_coefficient = 2.6e-7;
  const double x_dist             = 0.08;
  const double y_dist             = 0.08;
  const double min_speed          = 0.0;
  const double max_speed          = 2200.0;
  const double time_constant      = 0.02;
  const double rotational_inertia = 6.62e-6;

  p.dynamics_params.model_params.motors_params =
      multirotor::model::Model<double, 4>::create_quadrotor_x_config(
          thrust_coefficient, torque_coefficient, x_dist, y_dist, min_speed, max_speed,
          time_constant, rotational_inertia);

  // Controller params Indi
  p.controller_params.indi_controller_params.inertia =
      p.dynamics_params.model_params.vehicle_inertia;
  auto mixing_matrix_6D_4rotors = multirotor::model::Model<double, 4>::compute_mixer_matrix<6>(
      p.dynamics_params.model_params.motors_params);
  p.controller_params.indi_controller_params.mixer_matrix_inverse =
      indi_controller::compute_quadrotor_mixer_matrix_inverse(mixing_matrix_6D_4rotors);
  p.controller_params.indi_controller_params.pid_params.Kp_gains = Eigen::Vector3d(9.0, 9.0, 9.0);
  p.controller_params.indi_controller_params.pid_params.Ki_gains = Eigen::Vector3d(3.0, 3.0, 3.0);
  p.controller_params.indi_controller_params.pid_params.Kd_gains = Eigen::Vector3d(0.3, 0.3, 0.3);
  p.controller_params.indi_controller_params.pid_params.alpha    = Eigen::Vector3d(0.6, 0.6, 0.6);
  p.controller_params.indi_controller_params.pid_params.antiwindup_cte =
      Eigen::Vector3d(1.0, 1.0, 1.0);

  // Controller params Acro
  p.controller_params.acro_controller_params.gravity = p.dynamics_params.model_params.gravity;
  p.controller_params.acro_controller_params.vehicle_mass =
      p.dynamics_params.model_params.vehicle_mass;
  p.controller_params.acro_controller_params.kp_rot = Eigen::Vector3d(20.0, 20.0, 8.0).asDiagonal();

  // Controller params Trajectory
  p.controller_params.trajectory_controller_params.pid_params.Kp_gains =
      Eigen::Vector3d(10.0, 10.0, 10.0);
  p.controller_params.trajectory_controller_params.pid_params.Ki_gains =
      Eigen::Vector3d(0.005, 0.005, 0.005);
  p.controller_params.trajectory_controller_params.pid_params.Kd_gains =
      Eigen::Vector3d(6.0, 6.0, 6.0);
  p.controller_params.trajectory_controller_params.pid_params.alpha =
      Eigen::Vector3d(0.0, 0.0, 0.0);
  p.controller_params.trajectory_controller_params.pid_params.antiwindup_cte =
      Eigen::Vector3d(1.0, 1.0, 1.0);
  p.controller_params.trajectory_controller_params.pid_params.proportional_saturation_flag = true;

  // Controller params Speed
  p.controller_params.velocity_controller_params.pid_params.Kp_gains =
      Eigen::Vector3d(0.5, 0.5, 0.5);
  p.controller_params.velocity_controller_params.pid_params.Ki_gains =
      Eigen::Vector3d(0.0, 0.0, 0.0);
  p.controller_params.velocity_controller_params.pid_params.Kd_gains =
      Eigen::Vector3d(0.0, 0.0, 0.0);

  // Controller params Position
  p.controller_params.position_controller_params.pid_params.Kp_gains =
      Eigen::Vector3d(0.5, 0.5, 0.5);
  p.controller_params.position_controller_params.pid_params.Ki_gains =
      Eigen::Vector3d(0.0, 0.0, 0.0);
  p.controller_params.position_controller_params.pid_params.Kd_gains =
      Eigen::Vector3d(0.0, 0.0, 0.0);

  // Imu params
  p.imu_params.gyro_noise_var                 = 0.003;
  p.imu_params.accel_noise_var                = 0.005;
  p.imu_params.gyro_bias_noise_autocorr_time  = 1.0e-7;
  p.imu_params.accel_bias_noise_autocorr_time = 1.0e-7;

  // Inertial odometry params
  p.inertial_odometry_params.alpha = 0.9;

  return p;
}

TEST(SimulatorFly, floor_collision) {
  SimulatorParams simulator_params = get_simulation_params_from_code();
  Simulator simulator              = Simulator(simulator_params);
  simulator.enable_floor_collision(0.0);
  simulator.arm();

  double dt = 0.001;
  for (int i = 0; i < 10000; i++) {
    simulator.update_controller(dt);
    simulator.update_dynamics(dt);
    simulator.update_imu(dt);
    simulator.update_inertial_odometry(dt);
    Eigen::Vector3d imu_angular_velocity, imu_acceleration;
    simulator.get_imu_measurement(imu_angular_velocity, imu_acceleration);
  }
  double epsilon = 0.001;
  EXPECT_NEAR(simulator.get_state().kinematics.position.z(), 0.0, epsilon);
  EXPECT_NEAR(simulator.get_odometry().position.z(), 0.0, epsilon);
}

TEST(SimulatorFly, floor_collision_motor_w) {
  SimulatorParams simulator_params = get_simulation_params_from_code();
  Simulator simulator              = Simulator(simulator_params);
  simulator.enable_floor_collision(0.0);
  simulator.arm();
  simulator.set_control_mode(ControlMode::MOTOR_W);

  double dt = 0.001;
  for (int i = 0; i < 10000; i++) {
    simulator.update_controller(dt);
    simulator.update_dynamics(dt);
    simulator.update_imu(dt);
    simulator.update_inertial_odometry(dt);
  }
  double epsilon = 0.001;
  EXPECT_NEAR(simulator.get_state().kinematics.position.z(), 0.0, epsilon);
  EXPECT_NEAR(simulator.get_odometry().position.z(), 0.0, epsilon);
}

TEST(SimulatorFly, floor_collision_trajectory) {
  SimulatorParams simulator_params = get_simulation_params_from_code();
  Simulator simulator              = Simulator(simulator_params);
  simulator.enable_floor_collision(0.0);
  simulator.arm();
  simulator.set_control_mode(ControlMode::TRAJECTORY);

  double dt = 0.001;
  for (int i = 0; i < 10000; i++) {
    simulator.update_controller(dt);
    simulator.update_dynamics(dt);
    simulator.update_imu(dt);
    simulator.update_inertial_odometry(dt);
  }
  double epsilon = 0.001;
  EXPECT_NEAR(simulator.get_state().kinematics.position.z(), 0.0, epsilon);
  EXPECT_NEAR(simulator.get_odometry().position.z(), 0.0, epsilon);
}

TEST(SimulatorFly, floor_collision_velocity) {
  SimulatorParams simulator_params = get_simulation_params_from_code();
  Simulator simulator              = Simulator(simulator_params);
  simulator.enable_floor_collision(0.0);
  simulator.arm();
  simulator.set_control_mode(ControlMode::VELOCITY);

  double dt = 0.001;
  for (int i = 0; i < 10000; i++) {
    simulator.update_controller(dt);
    simulator.update_dynamics(dt);
    simulator.update_imu(dt);
    simulator.update_inertial_odometry(dt);
  }
  double epsilon = 0.001;
  EXPECT_NEAR(simulator.get_state().kinematics.position.z(), 0.0, epsilon);
  EXPECT_NEAR(simulator.get_odometry().position.z(), 0.0, epsilon);
}

TEST(SimulatorFly, floor_collision_position) {
  SimulatorParams simulator_params = get_simulation_params_from_code();
  Simulator simulator              = Simulator(simulator_params);
  simulator.enable_floor_collision(0.0);
  simulator.arm();
  simulator.set_control_mode(ControlMode::POSITION);

  double dt = 0.001;
  for (int i = 0; i < 10000; i++) {
    simulator.update_controller(dt);
    simulator.update_dynamics(dt);
    simulator.update_imu(dt);
    simulator.update_inertial_odometry(dt);
  }
  double epsilon = 0.001;
  EXPECT_NEAR(simulator.get_state().kinematics.position.z(), 0.0, epsilon);
  EXPECT_NEAR(simulator.get_odometry().position.z(), 0.0, epsilon);
}

TEST(SimulatorFly, low_dt) {
  SimulatorParams simulator_params = get_simulation_params_from_code();
  Simulator simulator              = Simulator(simulator_params);
  simulator.enable_floor_collision(0.0);
  simulator.arm();
  simulator.set_control_mode(ControlMode::POSITION);

  double dt = 0.01;
  for (int i = 0; i < 10000; i++) {
    simulator.update_controller(dt);
    simulator.update_dynamics(dt);
    simulator.update_imu(dt);
    simulator.update_inertial_odometry(dt);
  }
  double epsilon = 0.001;
  EXPECT_NEAR(simulator.get_state().kinematics.position.z(), 0.0, epsilon);
  EXPECT_NEAR(simulator.get_odometry().position.z(), 0.0, epsilon);
}

}  // namespace multirotor

int main(int argc, char* argv[]) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
