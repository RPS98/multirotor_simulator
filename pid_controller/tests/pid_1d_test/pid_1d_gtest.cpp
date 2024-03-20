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

#include "pid_controller/pid_1d.hpp"

namespace pid_1d_controller {

class PIDTest : public ::testing::Test {
protected:
  void SetUp() override {
    Kp_gains                = 1.0;
    Ki_gains                = 0.0;
    Kd_gains                = 0.0;
    antiwindup_cte          = 0.0;
    alpha                   = 1.0;
    reset_integral_flag     = false;
    upper_output_saturation = 0.0;
    lower_output_saturation = 0.0;

    pid_params.Kp_gains                = Kp_gains;
    pid_params.Ki_gains                = Ki_gains;
    pid_params.Kd_gains                = Kd_gains;
    pid_params.antiwindup_cte          = antiwindup_cte;
    pid_params.alpha                   = alpha;
    pid_params.reset_integral_flag     = reset_integral_flag;
    pid_params.upper_output_saturation = upper_output_saturation;
    pid_params.lower_output_saturation = lower_output_saturation;

    pid = std::make_shared<PID<double>>(pid_params);
    reset_controller();
  }

  void reset_controller() {
    pid->reset_controller();

    // First run
    state     = 0.0;
    reference = 0.0;
    error     = pid->get_error(state, reference);
    output    = pid->compute_control(dt, error);
  }

  // PID

  PIDParams<double> pid_params;
  std::shared_ptr<PID<double>> pid;

  // PID params
  double Kp_gains                = 1.0;
  double Ki_gains                = 0.0;
  double Kd_gains                = 0.0;
  double antiwindup_cte          = 0.0;
  double alpha                   = 1.0;
  bool reset_integral_flag       = false;
  double upper_output_saturation = 0.0;
  double lower_output_saturation = 0.0;

  // Declare variables needed for testing
  double state     = 0.0;
  double reference = 0.0;
  double error     = 0.0;
  double output    = 0.0;
  double dt        = 0.1;
};

TEST_F(PIDTest, default_constructors) {
  EXPECT_NO_THROW(PID pid_out = PID());
  PIDParams pid_params_out;
  EXPECT_NO_THROW(PID pid_out2 = PID(pid_params_out));
}

TEST_F(PIDTest, constructor) {
  // Check public methods
  PIDParams<double> pid_params_out = pid->get_params();
  EXPECT_EQ(pid_params_out.Kp_gains, pid_params.Kp_gains);
  EXPECT_EQ(pid_params_out.Ki_gains, pid_params.Ki_gains);
  EXPECT_EQ(pid_params_out.Kd_gains, pid_params.Kd_gains);
  EXPECT_EQ(pid_params_out.antiwindup_cte, pid_params.antiwindup_cte);
  EXPECT_EQ(pid_params_out.alpha, pid_params.alpha);
  EXPECT_EQ(pid_params_out.reset_integral_flag, pid_params.reset_integral_flag);
  EXPECT_EQ(pid_params_out.upper_output_saturation, pid_params.upper_output_saturation);

  PID pid_out(pid_params_out);
  EXPECT_EQ(pid_out.get_params().Kp_gains, pid_params_out.Kp_gains);
  EXPECT_EQ(pid_out.get_params().Ki_gains, pid_params_out.Ki_gains);
  EXPECT_EQ(pid_out.get_params().Kd_gains, pid_params_out.Kd_gains);
  EXPECT_EQ(pid_out.get_params().antiwindup_cte, pid_params_out.antiwindup_cte);
  EXPECT_EQ(pid_out.get_params().alpha, pid_params_out.alpha);
  EXPECT_EQ(pid_out.get_params().reset_integral_flag, pid_params_out.reset_integral_flag);
  EXPECT_EQ(pid_out.get_params().upper_output_saturation, pid_params_out.upper_output_saturation);
}

TEST_F(PIDTest, constructor_saturation_check) {
  EXPECT_FALSE(pid->get_output_saturation_flag());
  double epsilon = std::numeric_limits<double>::epsilon() * 0.1;

  // Check saturation
  pid_params.upper_output_saturation = 1.0;
  pid_params.lower_output_saturation = -1.0;
  pid->update_params(pid_params);
  EXPECT_TRUE(pid->get_output_saturation_flag());

  // Check saturation both positive
  pid_params.upper_output_saturation = 2.0;
  pid_params.lower_output_saturation = 1.0;
  pid->update_params(pid_params);
  EXPECT_TRUE(pid->get_output_saturation_flag());

  // Check saturation both negative
  pid_params.upper_output_saturation = -1.0;
  pid_params.lower_output_saturation = -2.0;
  pid->update_params(pid_params);
  EXPECT_TRUE(pid->get_output_saturation_flag());

  // Check saturation both close to zero
  pid_params.upper_output_saturation = epsilon;
  pid_params.lower_output_saturation = -epsilon;
  pid->update_params(pid_params);
  EXPECT_FALSE(pid->get_output_saturation_flag());

  // Check saturation both positive and close between them
  pid_params.upper_output_saturation = 1.0;
  pid_params.lower_output_saturation = 1.0 + epsilon;
  pid->update_params(pid_params);
  EXPECT_FALSE(pid->get_output_saturation_flag());

  // Check saturation both negative and close between them
  pid_params.upper_output_saturation = -1.0;
  pid_params.lower_output_saturation = -1.0 - epsilon;
  pid->update_params(pid_params);
  EXPECT_FALSE(pid->get_output_saturation_flag());
}

TEST_F(PIDTest, public_methods) {
  state = 0.0;

  // Check positive error
  reference = 1.0;
  error     = pid->get_error(state, reference);
  EXPECT_NO_THROW(pid->compute_control(dt, error));
  EXPECT_NO_THROW(pid->compute_control(dt, error, error));

  output = pid->compute_control(dt, error);
  EXPECT_EQ(output, pid_params.Kp_gains * error);
}

TEST_F(PIDTest, compute_control_kp) {
  state = 0.0;

  // Check positive error
  reference = 1.0;
  error     = pid->get_error(state, reference);
  output    = pid->compute_control(dt, error);
  EXPECT_EQ(output, pid_params.Kp_gains * error);

  // Check negative error
  reference = -1.0;
  error     = pid->get_error(state, reference);
  output    = pid->compute_control(dt, error);
  EXPECT_EQ(output, pid_params.Kp_gains * error);

  // Check zero error
  reference = 0.0;
  error     = pid->get_error(state, reference);
  output    = pid->compute_control(dt, error);
  EXPECT_EQ(output, pid_params.Kp_gains * error);

  // Check saturation
  pid_params.upper_output_saturation = 2.0;
  pid_params.lower_output_saturation = -2.0;
  pid->update_params(pid_params);

  reference = 3.0;
  error     = pid->get_error(state, reference);
  output    = pid->compute_control(dt, error);
  EXPECT_EQ(output, pid_params.upper_output_saturation);

  reference = -3.0;
  error     = pid->get_error(state, reference);
  output    = pid->compute_control(dt, error);
  EXPECT_EQ(output, pid_params.lower_output_saturation);
}

TEST_F(PIDTest, compute_control_ki) {
  Kp_gains            = 0.0;
  Ki_gains            = 1.0;
  Kd_gains            = 0.0;
  pid_params.Kp_gains = Kp_gains;
  pid_params.Ki_gains = Ki_gains;
  pid_params.Kd_gains = Kd_gains;
  pid->update_params(pid_params);

  state = 0.0;

  // Check positive error
  reference = 1.0;
  error     = pid->get_error(state, reference);
  output    = pid->compute_control(dt, error);
  EXPECT_EQ(output, pid_params.Ki_gains * error * dt);
  reset_controller();

  // Check negative error
  reference = -1.0;
  error     = pid->get_error(state, reference);
  output    = pid->compute_control(dt, error);
  EXPECT_EQ(output, pid_params.Ki_gains * error * dt);
  reset_controller();

  // Check zero error
  reference = 0.0;
  error     = pid->get_error(state, reference);
  output    = pid->compute_control(dt, error);
  EXPECT_EQ(output, pid_params.Ki_gains * error * dt);
  reset_controller();

  // Check saturation
  dt                        = 1.0;
  antiwindup_cte            = 2.0;
  pid_params.antiwindup_cte = antiwindup_cte;
  pid->update_params(pid_params);

  reference = 3.0;
  error     = pid->get_error(state, reference);
  output    = pid->compute_control(dt, error);
  EXPECT_EQ(output, antiwindup_cte);
  reset_controller();

  reference = -3.0;
  error     = pid->get_error(state, reference);
  output    = pid->compute_control(dt, error);
  EXPECT_EQ(output, -antiwindup_cte);
  reset_controller();
}

TEST_F(PIDTest, compute_control_kd) {
  Kp_gains            = 0.0;
  Ki_gains            = 0.0;
  Kd_gains            = 1.0;
  pid_params.Kp_gains = Kp_gains;
  pid_params.Ki_gains = Ki_gains;
  pid_params.Kd_gains = Kd_gains;
  pid->update_params(pid_params);

  state = 0.0;

  // Check positive error
  reference = 1.0;
  error     = pid->get_error(state, reference);
  output    = pid->compute_control(dt, error);
  EXPECT_EQ(output, pid_params.Kd_gains * error / dt);
  reset_controller();

  // Check negative error
  reference = -1.0;
  error     = pid->get_error(state, reference);
  output    = pid->compute_control(dt, error);
  EXPECT_EQ(output, pid_params.Kd_gains * error / dt);
  reset_controller();

  // Check zero error
  reference = 0.0;
  error     = pid->get_error(state, reference);
  output    = pid->compute_control(dt, error);
  EXPECT_EQ(output, pid_params.Kd_gains * error / dt);
  reset_controller();
}

}  // namespace pid_1d_controller

int main(int argc, char *argv[]) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
