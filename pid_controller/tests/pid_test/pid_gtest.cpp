/*!*******************************************************************************************
 *  \file       pid_1.hpp
 *  \brief      PID 1D Controller definition
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

#include "pid_controller/pid.hpp"

namespace pid_controller {

using Vector = Eigen::Vector3d;
using Matrix = Eigen::Matrix3d;

class PIDTest : public ::testing::Test {
protected:
  void SetUp() override {
    Kp_gains                     = Vector::Ones();
    Ki_gains                     = Vector::Zero();
    Kd_gains                     = Vector::Zero();
    antiwindup_cte               = Vector::Zero();
    alpha                        = Vector::Ones();
    reset_integral_flag          = false;
    proportional_saturation_flag = true;
    upper_output_saturation      = Vector::Zero();
    lower_output_saturation      = Vector::Zero();

    pid_params.Kp_gains                     = Kp_gains;
    pid_params.Ki_gains                     = Ki_gains;
    pid_params.Kd_gains                     = Kd_gains;
    pid_params.antiwindup_cte               = antiwindup_cte;
    pid_params.alpha                        = alpha;
    pid_params.reset_integral_flag          = reset_integral_flag;
    pid_params.proportional_saturation_flag = proportional_saturation_flag;
    pid_params.upper_output_saturation      = upper_output_saturation;
    pid_params.lower_output_saturation      = lower_output_saturation;

    pid = std::make_shared<PID<double, 3>>(pid_params);
    reset_controller();
  }

  void reset_controller() {
    pid->reset_controller();

    // First run
    state     = Vector::Zero();
    reference = Vector::Zero();
    error     = pid->get_error(state, reference);
    output    = pid->compute_control(dt, error);
  }

  // PID

  PIDParams<double, 3> pid_params;
  std::shared_ptr<PID<double, 3>> pid;

  // PID params
  Vector Kp_gains                   = Vector::Ones();
  Vector Ki_gains                   = Vector::Zero();
  Vector Kd_gains                   = Vector::Zero();
  Vector antiwindup_cte             = Vector::Zero();
  Vector alpha                      = Vector::Ones();
  bool reset_integral_flag          = false;
  bool proportional_saturation_flag = true;
  Vector upper_output_saturation    = Vector::Zero();
  Vector lower_output_saturation    = Vector::Zero();

  // Declare variables needed for testing
  Vector state     = Vector::Zero();
  Vector reference = Vector::Zero();
  Vector error     = Vector::Zero();
  Vector output    = Vector::Zero();
  double dt        = 0.1;
};

TEST_F(PIDTest, default_constructors) {
  EXPECT_NO_THROW(PID pid_out = PID());
  PIDParams pid_params_out;
  EXPECT_NO_THROW(PID pid_out2 = PID(pid_params_out));
}

TEST_F(PIDTest, constructor) {
  // Check public methods
  PIDParams<double, 3> pid_params_out = pid->get_params();
  EXPECT_EQ(pid_params_out.Kp_gains, pid_params.Kp_gains);
  EXPECT_EQ(pid_params_out.Ki_gains, pid_params.Ki_gains);
  EXPECT_EQ(pid_params_out.Kd_gains, pid_params.Kd_gains);
  EXPECT_EQ(pid_params_out.antiwindup_cte, pid_params.antiwindup_cte);
  EXPECT_EQ(pid_params_out.alpha, pid_params.alpha);
  EXPECT_EQ(pid_params_out.reset_integral_flag, pid_params.reset_integral_flag);
  EXPECT_EQ(pid_params_out.proportional_saturation_flag, pid_params.proportional_saturation_flag);
  EXPECT_EQ(pid_params_out.upper_output_saturation, pid_params.upper_output_saturation);

  PID<double, 3> pid_out(pid_params_out);
  EXPECT_EQ(pid_out.get_params().Kp_gains, pid_params_out.Kp_gains);
  EXPECT_EQ(pid_out.get_params().Ki_gains, pid_params_out.Ki_gains);
  EXPECT_EQ(pid_out.get_params().Kd_gains, pid_params_out.Kd_gains);
  EXPECT_EQ(pid_out.get_params().antiwindup_cte, pid_params_out.antiwindup_cte);
  EXPECT_EQ(pid_out.get_params().alpha, pid_params_out.alpha);
  EXPECT_EQ(pid_out.get_params().reset_integral_flag, pid_params_out.reset_integral_flag);
  EXPECT_EQ(pid_out.get_params().proportional_saturation_flag,
            pid_params_out.proportional_saturation_flag);
  EXPECT_EQ(pid_out.get_params().upper_output_saturation, pid_params_out.upper_output_saturation);
}

TEST_F(PIDTest, constructor_saturation_check) {
  EXPECT_FALSE(pid->get_output_saturation_flag());
  double epsilon = std::numeric_limits<double>::epsilon() * 0.1;

  // Check saturation
  pid_params.upper_output_saturation = Vector::Ones();
  pid_params.lower_output_saturation = -Vector::Ones();
  pid->update_pid_params(pid_params);
  EXPECT_TRUE(pid->get_output_saturation_flag());

  // Check saturation both positive
  pid_params.upper_output_saturation = 2.0 * Vector::Ones();
  pid_params.lower_output_saturation = Vector::Ones();
  pid->update_pid_params(pid_params);
  EXPECT_TRUE(pid->get_output_saturation_flag());

  // Check saturation both negative
  pid_params.upper_output_saturation = -Vector::Ones();
  pid_params.lower_output_saturation = -2.0 * Vector::Ones();
  pid->update_pid_params(pid_params);
  EXPECT_TRUE(pid->get_output_saturation_flag());

  // Check saturation both close to zero
  pid_params.upper_output_saturation = epsilon * Vector::Ones();
  pid_params.lower_output_saturation = -epsilon * Vector::Ones();
  pid->update_pid_params(pid_params);
  EXPECT_FALSE(pid->get_output_saturation_flag());

  // Check saturation both positive and close between them
  pid_params.upper_output_saturation = Vector::Ones();
  pid_params.lower_output_saturation = Vector::Ones() + Vector::Ones() * epsilon;
  pid->update_pid_params(pid_params);
  EXPECT_FALSE(pid->get_output_saturation_flag());

  // Check saturation both negative and close between them
  pid_params.upper_output_saturation = -Vector::Ones();
  pid_params.lower_output_saturation = -Vector::Ones() - Vector::Ones() * epsilon;
  pid->update_pid_params(pid_params);
  EXPECT_FALSE(pid->get_output_saturation_flag());
}

TEST_F(PIDTest, public_methods) {
  state = Eigen::Vector3d::Zero();

  // Check positive error
  reference = Eigen::Vector3d::Ones();
  error     = pid->get_error(state, reference);
  EXPECT_NO_THROW(pid->compute_control(dt, error));
  EXPECT_NO_THROW(pid->compute_control(dt, error, error));

  output = pid->compute_control(dt, error);
  EXPECT_EQ(output, pid_params.Kp_gains.asDiagonal() * error);
}

TEST_F(PIDTest, compute_control_kp) {
  state = Eigen::Vector3d::Zero();

  // Check positive error
  reference = Eigen::Vector3d::Ones();
  error     = pid->get_error(state, reference);
  output    = pid->compute_control(dt, error);
  EXPECT_EQ(output, pid_params.Kp_gains.asDiagonal() * error);

  // Check negative error
  reference = -Eigen::Vector3d::Ones();
  error     = pid->get_error(state, reference);
  output    = pid->compute_control(dt, error);
  EXPECT_EQ(output, pid_params.Kp_gains.asDiagonal() * error);

  // Check zero error
  reference = Eigen::Vector3d::Zero();
  error     = pid->get_error(state, reference);
  output    = pid->compute_control(dt, error);
  EXPECT_EQ(output, pid_params.Kp_gains.asDiagonal() * error);

  // Check saturation
  pid_params.proportional_saturation_flag = true;
  pid_params.upper_output_saturation      = Vector::Ones() * 2.0;
  pid_params.lower_output_saturation      = Vector::Ones() * -2.0;
  pid->update_pid_params(pid_params);

  reference = Eigen::Vector3d::Ones() * 3.0;
  error     = pid->get_error(state, reference);
  output    = pid->compute_control(dt, error);
  EXPECT_EQ(output, pid_params.upper_output_saturation);

  reference = Eigen::Vector3d::Ones() * -3.0;
  error     = pid->get_error(state, reference);
  output    = pid->compute_control(dt, error);
  EXPECT_EQ(output, pid_params.lower_output_saturation);

  reference = Eigen::Vector3d(1.0, 3.0, -4.0);
  error     = pid->get_error(state, reference);
  output    = pid->compute_control(dt, error);
  EXPECT_EQ(output, Eigen::Vector3d(0.5, 1.5, -2.0));
}

TEST_F(PIDTest, compute_control_ki) {
  Kp_gains            = Vector::Zero();
  Ki_gains            = Vector::Ones();
  Kd_gains            = Vector::Zero();
  pid_params.Kp_gains = Kp_gains;
  pid_params.Ki_gains = Ki_gains;
  pid_params.Kd_gains = Kd_gains;
  pid->update_pid_params(pid_params);

  state = Eigen::Vector3d::Zero();

  // Check positive error
  reference = Eigen::Vector3d::Ones();
  error     = pid->get_error(state, reference);
  output    = pid->compute_control(dt, error);
  EXPECT_EQ(output, pid_params.Ki_gains.asDiagonal() * error * dt);
  reset_controller();

  // Check negative error
  reference = -Eigen::Vector3d::Ones();
  error     = pid->get_error(state, reference);
  output    = pid->compute_control(dt, error);
  EXPECT_EQ(output, pid_params.Ki_gains.asDiagonal() * error * dt);
  reset_controller();

  // Check zero error
  reference = Eigen::Vector3d::Zero();
  error     = pid->get_error(state, reference);
  output    = pid->compute_control(dt, error);
  EXPECT_EQ(output, pid_params.Ki_gains.asDiagonal() * error * dt);
  reset_controller();

  // Check saturation
  dt                        = 1.0;
  antiwindup_cte            = Vector::Ones() * 2.0;
  pid_params.antiwindup_cte = antiwindup_cte;
  pid->update_pid_params(pid_params);

  reference = Eigen::Vector3d::Ones() * 3.0;
  error     = pid->get_error(state, reference);
  output    = pid->compute_control(dt, error);
  EXPECT_EQ(output, antiwindup_cte);
  reset_controller();

  reference = Eigen::Vector3d::Ones() * -3.0;
  error     = pid->get_error(state, reference);
  output    = pid->compute_control(dt, error);
  EXPECT_EQ(output, -antiwindup_cte);
  reset_controller();
}

TEST_F(PIDTest, compute_control_kd) {
  Kp_gains            = Vector::Zero();
  Ki_gains            = Vector::Zero();
  Kd_gains            = Vector::Ones();
  pid_params.Kp_gains = Kp_gains;
  pid_params.Ki_gains = Ki_gains;
  pid_params.Kd_gains = Kd_gains;
  pid->update_pid_params(pid_params);

  state = Eigen::Vector3d::Zero();

  // Check positive error
  reference = Eigen::Vector3d::Ones();
  error     = pid->get_error(state, reference);
  output    = pid->compute_control(dt, error);
  EXPECT_EQ(output, pid_params.Kd_gains.asDiagonal() * error / dt);
  reset_controller();

  // Check negative error
  reference = -Eigen::Vector3d::Ones();
  error     = pid->get_error(state, reference);
  output    = pid->compute_control(dt, error);
  EXPECT_EQ(output, pid_params.Kd_gains.asDiagonal() * error / dt);
  reset_controller();

  // Check zero error
  reference = Eigen::Vector3d::Zero();
  error     = pid->get_error(state, reference);
  output    = pid->compute_control(dt, error);
  EXPECT_EQ(output, pid_params.Kd_gains.asDiagonal() * error / dt);
  reset_controller();
}

}  // namespace pid_controller

int main(int argc, char *argv[]) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
