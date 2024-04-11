/*!*******************************************************************************************
 *  \file       indi_controller_gtest.hpp
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

#include "multirotor_controllers/controllers/indi_controller.hpp"

namespace indi_controller {

pid_controller::PIDParams<> get_pid_params() {
  pid_controller::PIDParams pid_params;
  pid_params.Kd_gains = Eigen::Vector3d::Ones() * 0.9;
  pid_params.Ki_gains = Eigen::Vector3d::Ones() * 3.0;
  pid_params.Kp_gains = Eigen::Vector3d::Ones() * 0.3;

  pid_params.antiwindup_cte      = Eigen::Vector3d::Ones() * 10.0;
  pid_params.alpha               = Eigen::Vector3d::Ones() * 0.1;
  pid_params.reset_integral_flag = false;

  pid_params.proportional_saturation_flag = true;
  pid_params.upper_output_saturation      = Eigen::Vector3d::Zero();
  pid_params.lower_output_saturation      = Eigen::Vector3d::Zero();
  return pid_params;
}

Eigen::Matrix3d get_vehicle_inertia() {
  // Inertia:
  // 0.0049      0      0
  //     0 0.0049      0
  //     0      0 0.0069
  return Eigen::Vector3d(0.0049, 0.0049, 0.0069).asDiagonal();
}

Eigen::Matrix<double, 4, 6> get_mixer_matrix_inverse(int motors_frame_type = 0) {
  Eigen::Matrix<double, 4, 6> mixer_matrix_inverse = Eigen::Matrix<double, 4, 6>::Zero();
  if (motors_frame_type == 0) {
    // Mixer matrix inverse:
    //          0            0       130890 -1.29771e-10 -3.27225e+06       961538
    //          0            0       130890  3.27225e+06  1.29771e-10      -961538
    //          0            0       130890  1.29771e-10  3.27225e+06       961538
    //          0            0       130890 -3.27225e+06 -1.29771e-10      -961538
    mixer_matrix_inverse << 0.0, 0.0, 130890.0, -1.29771e-10, -3.27225e+06, 961538.0, 0.0, 0.0,
        130890.0, 3.27225e+06, 1.29771e-10, -961538.0, 0.0, 0.0, 130890.0, 1.29771e-10, 3.27225e+06,
        961538.0, 0.0, 0.0, 130890.0, -3.27225e+06, -1.29771e-10, -961538.0;

  } else {
    // Mixer matrix inverse:
    //        0            0       130890 -1.63613e+06 -1.63613e+06       961538
    //        0            0       130890  1.63613e+06 -1.63613e+06      -961538
    //        0            0       130890  1.63613e+06  1.63613e+06       961538
    //        0            0       130890 -1.63613e+06  1.63613e+06      -961538
    mixer_matrix_inverse << 0.0, 0.0, 130890.0, -1.63613e+06, -1.63613e+06, 961538.0, 0.0, 0.0,
        130890.0, 1.63613e+06, -1.63613e+06, -961538.0, 0.0, 0.0, 130890.0, 1.63613e+06,
        1.63613e+06, 961538.0, 0.0, 0.0, 130890.0, -1.63613e+06, 1.63613e+06, -961538.0;
  }
  return mixer_matrix_inverse;
}

TEST(INDIController, constructor) {
  EXPECT_NO_THROW(IndiController indi_controller = IndiController());
  pid_controller::PIDParams pid_params = get_pid_params();
  EXPECT_NO_THROW(IndiController indi_controller = IndiController(
                      get_vehicle_inertia(), get_mixer_matrix_inverse(), pid_params));
  IndiControllerParams indi_controller_params;
  EXPECT_NO_THROW(IndiController indi_controller = IndiController(indi_controller_params));
}

TEST(INDIController, public_methods) {
  pid_controller::PIDParams pid_params = get_pid_params();
  IndiController indi_controller =
      IndiController(get_vehicle_inertia(), get_mixer_matrix_inverse(), pid_params);

  // Test the public methods
  // EXPECT_NO_THROW(indi_controller.update_model(model));
  EXPECT_NO_THROW(indi_controller.update_pid_params(pid_params));
  EXPECT_NO_THROW(indi_controller.reset_controller());
  Eigen::Vector3d current_angular_velocity = Eigen::Vector3d::Zero();
  double thrust                            = 1.0;
  Eigen::Vector3d desired_angular_velocity = Eigen::Vector3d::Zero();
  double dt                                = 0.01;
  EXPECT_NO_THROW(indi_controller.acro_to_motor_angular_velocity(current_angular_velocity, thrust,
                                                                 desired_angular_velocity, dt));
  Eigen::Matrix<double, 6, 4> mixer_matrix = Eigen::Matrix<double, 6, 4>::Zero();

  // Getters
  EXPECT_NO_THROW(indi_controller.get_inertia());
  EXPECT_NO_THROW(indi_controller.get_mixer_matrix_inverse());
  EXPECT_NO_THROW(indi_controller.get_desired_angular_acceleration());
  EXPECT_NO_THROW(indi_controller.get_desired_thrust());
  EXPECT_NO_THROW(indi_controller.get_desired_torque());
  EXPECT_NO_THROW(indi_controller.get_motor_angular_velocity());
  EXPECT_NO_THROW(indi_controller.get_angular_velocity_error());
}

TEST(INDIController, compute_quadrotor_mixer_matrix_inverse) {
  // [Fz]
  // [Tx] = mixer_matrix * [wn²]
  // [Ty]
  // [Tz]

  //                              [Fz]
  // [wn²] = mixer_matrix_inverse [Tx]
  //                              [Ty]
  //                              [Tz]

  double thrust_coefficient = 1.91e-6;
  double torque_coefficient = 2.6e-7;
  double x_dist             = 0.08;
  double y_dist             = 0.08;

  // Mixer matrix:
  Eigen::Matrix<double, 4, 4> mixer_matrix_4d = Eigen::Matrix<double, 4, 4>::Zero();
  mixer_matrix_4d << thrust_coefficient, thrust_coefficient, thrust_coefficient, thrust_coefficient,
      -y_dist * thrust_coefficient, y_dist * thrust_coefficient, y_dist * thrust_coefficient,
      -y_dist * thrust_coefficient, -x_dist * thrust_coefficient, -x_dist * thrust_coefficient,
      x_dist * thrust_coefficient, x_dist * thrust_coefficient, torque_coefficient,
      -torque_coefficient, torque_coefficient, -torque_coefficient;

  // Mixer matrix inverse:
  Eigen::Matrix<double, 4, 4> mixer_matrix_4d_inverse = mixer_matrix_4d.inverse();

  // Mixer matrix extended:
  Eigen::Matrix<double, 6, 4> mixer_matrix = Eigen::Matrix<double, 6, 4>::Zero();
  mixer_matrix << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, thrust_coefficient, thrust_coefficient,
      thrust_coefficient, thrust_coefficient, -y_dist * thrust_coefficient,
      y_dist * thrust_coefficient, y_dist * thrust_coefficient, -y_dist * thrust_coefficient,
      -x_dist * thrust_coefficient, -x_dist * thrust_coefficient, x_dist * thrust_coefficient,
      x_dist * thrust_coefficient, torque_coefficient, -torque_coefficient, torque_coefficient,
      -torque_coefficient;

  // Mixer matrix inverse extended computed:
  Eigen::Matrix<double, 4, 6> mixer_matrix_inverse_computed =
      indi_controller::compute_quadrotor_mixer_matrix_inverse(mixer_matrix);

  // Mixer matrix inverse extended expected:
  Eigen::Matrix<double, 4, 6> mixer_matrix_expected;
  mixer_matrix_expected.col(0) = Eigen::Matrix<double, 4, 1>::Zero();
  mixer_matrix_expected.col(1) = Eigen::Matrix<double, 4, 1>::Zero();
  mixer_matrix_expected.col(2) = mixer_matrix_4d_inverse.col(0);
  mixer_matrix_expected.col(3) = mixer_matrix_4d_inverse.col(1);
  mixer_matrix_expected.col(4) = mixer_matrix_4d_inverse.col(2);
  mixer_matrix_expected.col(5) = mixer_matrix_4d_inverse.col(3);

  // Check the computed and expected mixer matrix inverse extended are equal
  EXPECT_EQ(mixer_matrix_inverse_computed, mixer_matrix_expected);

  // Checl warning std::cerr
  mixer_matrix << 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, thrust_coefficient, thrust_coefficient,
      thrust_coefficient, thrust_coefficient, -y_dist * thrust_coefficient,
      y_dist * thrust_coefficient, y_dist * thrust_coefficient, -y_dist * thrust_coefficient,
      -x_dist * thrust_coefficient, -x_dist * thrust_coefficient, x_dist * thrust_coefficient,
      x_dist * thrust_coefficient, torque_coefficient, -torque_coefficient, torque_coefficient,
      -torque_coefficient;

  testing::internal::CaptureStderr();
  mixer_matrix_inverse_computed =
      indi_controller::compute_quadrotor_mixer_matrix_inverse(mixer_matrix);
  std::string output = testing::internal::GetCapturedStderr();
  EXPECT_EQ(output,
            "Warning - compute_mixer_matrix_inverse: first two rows of quadrotor matrix must be "
            "zeros. If not, try to compute pseudo-inverse.\n");
}

TEST(INDIController, acro_to_motor_speeds_config_plus) {
  pid_controller::PIDParams pid_params = get_pid_params();
  IndiController indi_controller =
      IndiController(get_vehicle_inertia(), get_mixer_matrix_inverse(), pid_params);

  // Input
  double thrust                            = 1.0;
  Eigen::Vector3d desired_angular_velocity = Eigen::Vector3d::Zero();
  Eigen::Vector3d current_angular_velocity = Eigen::Vector3d::Zero();

  // Output
  Eigen::Vector4d output_motor_angular_velocity;

  double speed = 1.0;
  double dt    = 0.01;

  // Hover / Move up / Move down
  desired_angular_velocity = Eigen::Vector3d(0.0, 0.0, 0.0);
  indi_controller.reset_controller();
  output_motor_angular_velocity = indi_controller.acro_to_motor_angular_velocity(
      current_angular_velocity, thrust, desired_angular_velocity, dt);
  EXPECT_EQ(output_motor_angular_velocity[0], output_motor_angular_velocity[1]);
  EXPECT_EQ(output_motor_angular_velocity[1], output_motor_angular_velocity[2]);
  EXPECT_EQ(output_motor_angular_velocity[2], output_motor_angular_velocity[3]);

  // Rotate clockwise
  desired_angular_velocity = Eigen::Vector3d(0.0, 0.0, -speed);
  indi_controller.reset_controller();
  output_motor_angular_velocity = indi_controller.acro_to_motor_angular_velocity(
      current_angular_velocity, thrust, desired_angular_velocity, dt);
  EXPECT_EQ(output_motor_angular_velocity[0], output_motor_angular_velocity[2]);
  EXPECT_EQ(output_motor_angular_velocity[1], output_motor_angular_velocity[3]);
  EXPECT_LT(output_motor_angular_velocity[0], output_motor_angular_velocity[1]);

  // Rotate counter-clockwise
  desired_angular_velocity = Eigen::Vector3d(0.0, 0.0, speed);
  indi_controller.reset_controller();
  output_motor_angular_velocity = indi_controller.acro_to_motor_angular_velocity(
      current_angular_velocity, thrust, desired_angular_velocity, dt);
  EXPECT_EQ(output_motor_angular_velocity[0], output_motor_angular_velocity[2]);
  EXPECT_EQ(output_motor_angular_velocity[1], output_motor_angular_velocity[3]);
  EXPECT_GT(output_motor_angular_velocity[0], output_motor_angular_velocity[1]);

  // Move forward
  desired_angular_velocity = Eigen::Vector3d(0.0, speed, 0.0);
  indi_controller.reset_controller();
  output_motor_angular_velocity = indi_controller.acro_to_motor_angular_velocity(
      current_angular_velocity, thrust, desired_angular_velocity, dt);
  EXPECT_LT(output_motor_angular_velocity[0], output_motor_angular_velocity[2]);
  EXPECT_EQ(output_motor_angular_velocity[1], output_motor_angular_velocity[3]);
  EXPECT_LT(output_motor_angular_velocity[0], output_motor_angular_velocity[1]);
  EXPECT_GT(output_motor_angular_velocity[2], output_motor_angular_velocity[1]);

  // Move backward
  desired_angular_velocity = Eigen::Vector3d(0.0, -speed, 0.0);
  indi_controller.reset_controller();
  output_motor_angular_velocity = indi_controller.acro_to_motor_angular_velocity(
      current_angular_velocity, thrust, desired_angular_velocity, dt);
  EXPECT_GT(output_motor_angular_velocity[0], output_motor_angular_velocity[2]);
  EXPECT_EQ(output_motor_angular_velocity[1], output_motor_angular_velocity[3]);
  EXPECT_GT(output_motor_angular_velocity[0], output_motor_angular_velocity[1]);
  EXPECT_LT(output_motor_angular_velocity[2], output_motor_angular_velocity[1]);

  // Move left
  desired_angular_velocity = Eigen::Vector3d(-speed, 0.0, 0.0);
  indi_controller.reset_controller();
  output_motor_angular_velocity = indi_controller.acro_to_motor_angular_velocity(
      current_angular_velocity, thrust, desired_angular_velocity, dt);
  EXPECT_LT(output_motor_angular_velocity[1], output_motor_angular_velocity[3]);
  EXPECT_EQ(output_motor_angular_velocity[0], output_motor_angular_velocity[2]);
  EXPECT_LT(output_motor_angular_velocity[1], output_motor_angular_velocity[0]);
  EXPECT_GT(output_motor_angular_velocity[3], output_motor_angular_velocity[0]);

  // Move right
  desired_angular_velocity = Eigen::Vector3d(speed, 0.0, 0.0);
  indi_controller.reset_controller();
  output_motor_angular_velocity = indi_controller.acro_to_motor_angular_velocity(
      current_angular_velocity, thrust, desired_angular_velocity, dt);
  EXPECT_GT(output_motor_angular_velocity[1], output_motor_angular_velocity[3]);
  EXPECT_EQ(output_motor_angular_velocity[0], output_motor_angular_velocity[2]);
  EXPECT_GT(output_motor_angular_velocity[1], output_motor_angular_velocity[0]);
  EXPECT_LT(output_motor_angular_velocity[3], output_motor_angular_velocity[0]);

  // Move diagonally forward left
  desired_angular_velocity = Eigen::Vector3d(-speed, speed, 0.0);
  indi_controller.reset_controller();
  output_motor_angular_velocity = indi_controller.acro_to_motor_angular_velocity(
      current_angular_velocity, thrust, desired_angular_velocity, dt);
  EXPECT_EQ(output_motor_angular_velocity[0], output_motor_angular_velocity[1]);
  EXPECT_EQ(output_motor_angular_velocity[2], output_motor_angular_velocity[3]);
  EXPECT_LT(output_motor_angular_velocity[0], output_motor_angular_velocity[2]);

  // Move diagonally forward right
  desired_angular_velocity = Eigen::Vector3d(speed, speed, 0.0);
  indi_controller.reset_controller();
  output_motor_angular_velocity = indi_controller.acro_to_motor_angular_velocity(
      current_angular_velocity, thrust, desired_angular_velocity, dt);
  EXPECT_EQ(output_motor_angular_velocity[0], output_motor_angular_velocity[3]);
  EXPECT_EQ(output_motor_angular_velocity[1], output_motor_angular_velocity[2]);
  EXPECT_LT(output_motor_angular_velocity[0], output_motor_angular_velocity[1]);

  // Move diagonally backward left
  desired_angular_velocity = Eigen::Vector3d(-speed, -speed, 0.0);
  indi_controller.reset_controller();
  output_motor_angular_velocity = indi_controller.acro_to_motor_angular_velocity(
      current_angular_velocity, thrust, desired_angular_velocity, dt);
  EXPECT_EQ(output_motor_angular_velocity[0], output_motor_angular_velocity[3]);
  EXPECT_EQ(output_motor_angular_velocity[1], output_motor_angular_velocity[2]);
  EXPECT_GT(output_motor_angular_velocity[0], output_motor_angular_velocity[1]);

  // Move diagonally backward right
  desired_angular_velocity = Eigen::Vector3d(speed, -speed, 0.0);
  indi_controller.reset_controller();
  output_motor_angular_velocity = indi_controller.acro_to_motor_angular_velocity(
      current_angular_velocity, thrust, desired_angular_velocity, dt);
  EXPECT_EQ(output_motor_angular_velocity[0], output_motor_angular_velocity[1]);
  EXPECT_EQ(output_motor_angular_velocity[2], output_motor_angular_velocity[3]);
  EXPECT_GT(output_motor_angular_velocity[0], output_motor_angular_velocity[2]);
}

TEST(INDIController, acro_to_motor_speeds_config_cross) {
  pid_controller::PIDParams pid_params = get_pid_params();
  IndiController indi_controller =
      IndiController(get_vehicle_inertia(), get_mixer_matrix_inverse(1), pid_params);

  // Input
  double thrust                            = 1.0;
  Eigen::Vector3d desired_angular_velocity = Eigen::Vector3d::Zero();
  Eigen::Vector3d current_angular_velocity = Eigen::Vector3d::Zero();

  // Output
  Eigen::Vector4d output_motor_angular_velocity;

  double speed = 1.0;
  double dt    = 0.01;

  // Move up
  desired_angular_velocity = Eigen::Vector3d(0.0, 0.0, 0.0);
  indi_controller.reset_controller();
  output_motor_angular_velocity = indi_controller.acro_to_motor_angular_velocity(
      current_angular_velocity, thrust, desired_angular_velocity, dt);
  EXPECT_EQ(output_motor_angular_velocity[0], output_motor_angular_velocity[1]);
  EXPECT_EQ(output_motor_angular_velocity[1], output_motor_angular_velocity[2]);
  EXPECT_EQ(output_motor_angular_velocity[2], output_motor_angular_velocity[3]);

  // Move clockwise
  desired_angular_velocity = Eigen::Vector3d(0.0, 0.0, -speed);
  indi_controller.reset_controller();
  output_motor_angular_velocity = indi_controller.acro_to_motor_angular_velocity(
      current_angular_velocity, thrust, desired_angular_velocity, dt);
  EXPECT_EQ(output_motor_angular_velocity[0], output_motor_angular_velocity[2]);
  EXPECT_EQ(output_motor_angular_velocity[1], output_motor_angular_velocity[3]);
  EXPECT_LT(output_motor_angular_velocity[0], output_motor_angular_velocity[1]);

  // Move counter-clockwise
  desired_angular_velocity = Eigen::Vector3d(0.0, 0.0, speed);
  indi_controller.reset_controller();
  output_motor_angular_velocity = indi_controller.acro_to_motor_angular_velocity(
      current_angular_velocity, thrust, desired_angular_velocity, dt);
  EXPECT_EQ(output_motor_angular_velocity[0], output_motor_angular_velocity[2]);
  EXPECT_EQ(output_motor_angular_velocity[1], output_motor_angular_velocity[3]);
  EXPECT_GT(output_motor_angular_velocity[0], output_motor_angular_velocity[1]);

  // Move forward
  desired_angular_velocity = Eigen::Vector3d(0.0, speed, 0.0);
  indi_controller.reset_controller();
  output_motor_angular_velocity = indi_controller.acro_to_motor_angular_velocity(
      current_angular_velocity, thrust, desired_angular_velocity, dt);
  EXPECT_EQ(output_motor_angular_velocity[0], output_motor_angular_velocity[1]);
  EXPECT_EQ(output_motor_angular_velocity[2], output_motor_angular_velocity[3]);
  EXPECT_LT(output_motor_angular_velocity[0], output_motor_angular_velocity[2]);

  // Move backward
  desired_angular_velocity = Eigen::Vector3d(0.0, -speed, 0.0);
  indi_controller.reset_controller();
  output_motor_angular_velocity = indi_controller.acro_to_motor_angular_velocity(
      current_angular_velocity, thrust, desired_angular_velocity, dt);
  EXPECT_EQ(output_motor_angular_velocity[0], output_motor_angular_velocity[1]);
  EXPECT_EQ(output_motor_angular_velocity[2], output_motor_angular_velocity[3]);
  EXPECT_GT(output_motor_angular_velocity[0], output_motor_angular_velocity[2]);

  // Move left
  desired_angular_velocity = Eigen::Vector3d(-speed, 0.0, 0.0);
  indi_controller.reset_controller();
  output_motor_angular_velocity = indi_controller.acro_to_motor_angular_velocity(
      current_angular_velocity, thrust, desired_angular_velocity, dt);
  EXPECT_EQ(output_motor_angular_velocity[0], output_motor_angular_velocity[3]);
  EXPECT_EQ(output_motor_angular_velocity[1], output_motor_angular_velocity[2]);
  EXPECT_GT(output_motor_angular_velocity[0], output_motor_angular_velocity[1]);

  // Move right
  desired_angular_velocity = Eigen::Vector3d(speed, 0.0, 0.0);
  indi_controller.reset_controller();
  output_motor_angular_velocity = indi_controller.acro_to_motor_angular_velocity(
      current_angular_velocity, thrust, desired_angular_velocity, dt);
  EXPECT_EQ(output_motor_angular_velocity[0], output_motor_angular_velocity[3]);
  EXPECT_EQ(output_motor_angular_velocity[1], output_motor_angular_velocity[2]);
  EXPECT_LT(output_motor_angular_velocity[0], output_motor_angular_velocity[1]);

  // Move diagonally forward left
  desired_angular_velocity = Eigen::Vector3d(-speed, speed, 0.0);
  indi_controller.reset_controller();
  output_motor_angular_velocity = indi_controller.acro_to_motor_angular_velocity(
      current_angular_velocity, thrust, desired_angular_velocity, dt);
  EXPECT_LT(output_motor_angular_velocity[1], output_motor_angular_velocity[3]);
  EXPECT_EQ(output_motor_angular_velocity[0], output_motor_angular_velocity[2]);
  EXPECT_LT(output_motor_angular_velocity[1], output_motor_angular_velocity[0]);
  EXPECT_GT(output_motor_angular_velocity[3], output_motor_angular_velocity[0]);

  // Move diagonally forward right
  desired_angular_velocity = Eigen::Vector3d(speed, speed, 0.0);
  indi_controller.reset_controller();
  output_motor_angular_velocity = indi_controller.acro_to_motor_angular_velocity(
      current_angular_velocity, thrust, desired_angular_velocity, dt);
  EXPECT_LT(output_motor_angular_velocity[0], output_motor_angular_velocity[2]);
  EXPECT_EQ(output_motor_angular_velocity[1], output_motor_angular_velocity[3]);
  EXPECT_LT(output_motor_angular_velocity[0], output_motor_angular_velocity[1]);
  EXPECT_GT(output_motor_angular_velocity[2], output_motor_angular_velocity[1]);

  // Move diagonally backward left
  desired_angular_velocity = Eigen::Vector3d(-speed, -speed, 0.0);
  indi_controller.reset_controller();
  output_motor_angular_velocity = indi_controller.acro_to_motor_angular_velocity(
      current_angular_velocity, thrust, desired_angular_velocity, dt);
  EXPECT_GT(output_motor_angular_velocity[0], output_motor_angular_velocity[2]);
  EXPECT_EQ(output_motor_angular_velocity[1], output_motor_angular_velocity[3]);
  EXPECT_GT(output_motor_angular_velocity[0], output_motor_angular_velocity[1]);
  EXPECT_LT(output_motor_angular_velocity[2], output_motor_angular_velocity[1]);

  // Move diagonally backward right
  desired_angular_velocity = Eigen::Vector3d(speed, -speed, 0.0);
  indi_controller.reset_controller();
  output_motor_angular_velocity = indi_controller.acro_to_motor_angular_velocity(
      current_angular_velocity, thrust, desired_angular_velocity, dt);
  EXPECT_GT(output_motor_angular_velocity[1], output_motor_angular_velocity[3]);
  EXPECT_EQ(output_motor_angular_velocity[0], output_motor_angular_velocity[2]);
  EXPECT_GT(output_motor_angular_velocity[1], output_motor_angular_velocity[0]);
  EXPECT_LT(output_motor_angular_velocity[3], output_motor_angular_velocity[0]);
}

}  // namespace indi_controller

int main(int argc, char *argv[]) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
