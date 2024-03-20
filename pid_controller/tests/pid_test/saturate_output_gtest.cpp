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

class SaturateOutputTest : public ::testing::Test {
protected:
  void SetUp() override {
    // Set up test vectors and limits
    output       = Vector::Ones() * 3.0;
    upper_limits = Vector::Ones() * 2.0;
    lower_limits = Vector::Ones() * -2.0;
  }

  // Declare variables needed for testing
  Vector output       = Vector::Zero();
  Vector upper_limits = Vector::Zero();
  Vector lower_limits = Vector::Zero();
};

TEST_F(SaturateOutputTest, NonProportionalSaturation) {
  Vector saturated = PID<double, 3>::saturate_output(output, upper_limits, lower_limits, false);

  // Test that the saturated values are within limits
  for (ptrdiff_t i = 0; i < saturated.size(); i++) {
    ASSERT_LE(saturated[i], upper_limits[i]);
    ASSERT_GE(saturated[i], lower_limits[i]);
  }
}

TEST_F(SaturateOutputTest, ProportionalSaturation) {
  Vector saturated = PID<double, 3>::saturate_output(output, upper_limits, lower_limits, true);

  // Test that the saturated values are within limits
  for (ptrdiff_t i = 0; i < saturated.size(); i++) {
    ASSERT_LE(saturated[i], upper_limits[i]);
    ASSERT_GE(saturated[i], lower_limits[i]);
  }

  // Test the direction of the vector
  // x-y angle
  double x                   = output[0];
  double y                   = output[1];
  double z                   = output[2];
  double x_y_angle           = std::atan2(y, x);
  double x_y_angle_saturated = std::atan2(saturated[1], saturated[0]);
  ASSERT_NEAR(x_y_angle, x_y_angle_saturated, 1e-6);

  // x-z angle
  double x_z_angle           = std::atan2(z, x);
  double x_z_angle_saturated = std::atan2(saturated[2], saturated[0]);
  ASSERT_NEAR(x_z_angle, x_z_angle_saturated, 1e-6);
}

}  // namespace pid_controller

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
