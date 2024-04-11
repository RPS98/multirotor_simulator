/*!*******************************************************************************************
 *  \file       utils_gtest.cpp
 *  \brief      Gtest
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
#include <iostream>
#include "multirotor_dynamic_model/common/utils.hpp"

namespace multirotor::utils {

// Check that the vector is clamped between min and max
TEST(Utils, clamp_vector) {
  // Float precision
  Eigen::Vector3f vector_to_clamp(1.5, 0.3, -2.0);
  float min_value = -1.0;
  float max_value = 1.0;

  clamp_vector(vector_to_clamp, min_value, max_value);

  EXPECT_FLOAT_EQ(vector_to_clamp.x(), 1.0);
  EXPECT_FLOAT_EQ(vector_to_clamp.y(), 0.3);
  EXPECT_FLOAT_EQ(vector_to_clamp.z(), -1.0);

  // Double precision
  Eigen::Vector3d vector_to_clamp_double(1.5, 0.3, -2.0);
  double min_value_double = -1.0;
  double max_value_double = 1.0;

  clamp_vector(vector_to_clamp_double, min_value_double, max_value_double);

  EXPECT_DOUBLE_EQ(vector_to_clamp_double.x(), 1.0);
  EXPECT_DOUBLE_EQ(vector_to_clamp_double.y(), 0.3);
  EXPECT_DOUBLE_EQ(vector_to_clamp_double.z(), -1.0);
}

// GTest for get_quaternion_derivative: check that the derivative is correct.
TEST(Utils, get_quaternion_derivative_precision) {
  // Float precision
  Eigen::Quaternionf q_f(1.0, 0.0, 0.0, 0.0);  // Identity quaternion
  Eigen::Vector3f omega_f(0.1, 0.2, 0.3);      // Some angular velocity

  Eigen::Vector4f q_dot_f = get_quaternion_derivative(q_f, omega_f);

  // Double precision
  Eigen::Quaterniond q_d(1.0, 0.0, 0.0, 0.0);  // Identity quaternion
  Eigen::Vector3d omega_d(0.1, 0.2, 0.3);      // Some angular velocity

  Eigen::Vector4d q_dot_d = get_quaternion_derivative(q_d, omega_d);
}

TEST(Utils, get_quaternion_derivative_values) {
  const double epsilon   = 1e-6;
  Eigen::Vector4d result = Eigen::Vector4d::Zero();
  Eigen::Vector3d omega(0.0, 0.0, 0.0);

  // Test for each quaternion initial value
  for (int i = 0; i < 10; ++i) {  // You can increase/decrease the number of tests here
    // Generate random quaternion values (normalized)
    Eigen::Quaterniond q = Eigen::Quaterniond::UnitRandom().normalized();

    // Test for each omega combination
    for (double x = -1.0; x <= 1.0; x += 0.1) {
      for (double y = -1.0; y <= 1.0; y += 0.1) {
        for (double z = -1.0; z <= 1.0; z += 0.1) {
          omega  = Eigen::Vector3d(x, y, z);
          result = get_quaternion_derivative(q, omega);
          ASSERT_NEAR(result[0], 0.5 * (q.w() * omega.x() - q.z() * omega.y() + q.y() * omega.z()),
                      epsilon);
          ASSERT_NEAR(result[1], 0.5 * (q.z() * omega.x() + q.w() * omega.y() - q.x() * omega.z()),
                      epsilon);
          ASSERT_NEAR(result[2], 0.5 * (-q.y() * omega.x() + q.x() * omega.y() + q.w() * omega.z()),
                      epsilon);
          ASSERT_NEAR(result[3], 0.5 * (-q.x() * omega.x() - q.y() * omega.y() - q.z() * omega.z()),
                      epsilon);
        }
      }
    }
  }
}

TEST(Utils, get_quaternion_integrate_precision) {
  // Float precision
  float dt_f = 0.01;
  Eigen::Quaternionf q_f(1.0, 0.0, 0.0, 0.0);  // Identity quaternion
  Eigen::Vector3f omega_f(0.1, 0.2, 0.3);      // Some angular velocity

  Eigen::Quaternionf q_int_f = get_quaternion_integrate(q_f, omega_f, dt_f);

  // Double precision
  double dt_d = 0.01;
  Eigen::Quaterniond q_d(1.0, 0.0, 0.0, 0.0);  // Identity quaternion
  Eigen::Vector3d omega_d(0.1, 0.2, 0.3);      // Some angular velocity

  Eigen::Quaterniond q_int_d = get_quaternion_integrate(q_d, omega_d, dt_d);
}

TEST(Utils, get_quaternion_integrate_values) {
  const double epsilon      = 0.01;
  Eigen::Quaterniond result = Eigen::Quaterniond::Identity();
  Eigen::Vector3d omega(0.0, 0.0, 0.0);
  const double dt = 0.001;  // Small time step to increase precision

  // Test for each quaternion initial value
  for (int i = 0; i < 10; ++i) {  // You can increase/decrease the number of tests here
    // Generate random quaternion values (normalized)
    Eigen::Quaterniond q = Eigen::Quaterniond::UnitRandom().normalized();

    // Test for each omega combination
    for (double x = -1.0; x <= 1.0; x += 0.1) {
      for (double y = -1.0; y <= 1.0; y += 0.1) {
        for (double z = -1.0; z <= 1.0; z += 0.1) {
          omega  = Eigen::Vector3d(x, y, z);
          result = get_quaternion_integrate(q, omega, dt);

          // Convert the integrated quaternion back to Euler angles
          Eigen::Vector3d integrated_euler_angles =
              result.toRotationMatrix().eulerAngles(0, 1, 2);  // XYZ order

          // Integrate the Euler angles directly using the angular velocity
          Eigen::Vector3d q_euler_angles = q.toRotationMatrix().eulerAngles(0, 1, 2);  // XYZ order
          Eigen::Vector3d euler_angles_integration =
              Eigen::Vector3d(q_euler_angles[0] + omega[0] * dt, q_euler_angles[1] + omega[1] * dt,
                              q_euler_angles[2] + omega[2] * dt);

          // Normalize Euler angles to the range [-pi, pi]
          euler_angles_integration[0] =
              std::fmod(euler_angles_integration[0] + M_PI, 2 * M_PI) - M_PI;
          euler_angles_integration[1] =
              std::fmod(euler_angles_integration[1] + M_PI, 2 * M_PI) - M_PI;
          euler_angles_integration[2] =
              std::fmod(euler_angles_integration[2] + M_PI, 2 * M_PI) - M_PI;

          // Check that the integrated Euler angles match within the tolerance
          ASSERT_NEAR(integrated_euler_angles[0], euler_angles_integration[0], epsilon);
          ASSERT_NEAR(integrated_euler_angles[1], euler_angles_integration[1], epsilon);
          ASSERT_NEAR(integrated_euler_angles[2], euler_angles_integration[2], epsilon);
        }
      }
    }
  }
}

TEST(Utils, squared_keep_sign_precision) {
  // Float precision
  Eigen::Vector3f vector_to_square(1.0, -2.0, 3.0);

  Eigen::Vector3f squared_vector_f = squared_keep_sign(vector_to_square);

  // Double precision
  Eigen::Vector3d vector_to_square_double(1.0, -2.0, 3.0);

  Eigen::Vector3d squared_vector_d = squared_keep_sign(vector_to_square_double);
}

TEST(Utils, squared_keep_sign_values) {
  const double epsilon = 1e-6;
  const int num_tests  = 100;  // Number of random tests

  for (int i = 0; i < num_tests; ++i) {
    Eigen::Vector3d vector_to_square = Eigen::Vector3d::Random() * 10.0;
    Eigen::Vector3d squared_vector   = squared_keep_sign(vector_to_square);

    for (int j = 0; j < 3; ++j) {
      double expected_result =
          vector_to_square[j] * vector_to_square[j] * std::copysign(1.0, vector_to_square[j]);
      ASSERT_NEAR(squared_vector[j], expected_result, epsilon);
    }
  }
}

TEST(Utils, sqrt_keep_sign_precision) {
  // Float precision
  Eigen::Vector3f vector_to_sqrt(1.0, 4.0, 9.0);
  Eigen::Vector3f sqrt_vector_f = sqrt_keep_sign(vector_to_sqrt);

  // Double precision
  Eigen::Vector3d vector_to_sqrt_double(1.0, 4.0, 9.0);
  Eigen::Vector3d sqrt_vector_d = sqrt_keep_sign(vector_to_sqrt_double);
}

TEST(Utils, sqrt_keep_sign_values) {
  const double epsilon = 1e-6;
  const int num_tests  = 100;  // Number of random tests

  for (int i = 0; i < num_tests; ++i) {
    Eigen::Vector3d vector_to_sqrt = Eigen::Vector3d::Random() * 10.0;
    Eigen::Vector3d sqrt_vector    = sqrt_keep_sign(vector_to_sqrt);

    for (int j = 0; j < 3; ++j) {
      double expected_result =
          std::sqrt(std::abs(vector_to_sqrt[j])) * std::copysign(1.0, vector_to_sqrt[j]);
      ASSERT_NEAR(sqrt_vector[j], expected_result, epsilon);
    }
  }
}

}  // namespace multirotor::utils

int main(int argc, char* argv[]) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
