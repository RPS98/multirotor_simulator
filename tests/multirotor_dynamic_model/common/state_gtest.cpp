/*!*******************************************************************************************
 *  \file       state_gtest.hpp
 *  \brief      Multirotor State struct unit tests.
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
#include "multirotor_dynamic_model/common/state.hpp"

template <typename P = double, int num_rotors = 4>
using State = multirotor::state::State<P, num_rotors>;
template <typename P = double>
using Kinematics = multirotor::state::internal::Kinematics<P>;
template <typename P = double>
using Dynamics = multirotor::state::internal::Dynamics<P>;
template <typename P = double, int num_rotors = 4>
using Actuators = multirotor::state::internal::Actuators<P, num_rotors>;

// Test the default constructor
TEST(State, Kinematics_constructor_default) {
  Kinematics<double> kinematics;

  // Check if all fields are initialized to zero
  EXPECT_EQ(kinematics.position, Eigen::Vector3d::Zero());
  // EXPECT_EQ(kinematics.orientation, Eigen::Quaterniond::Identity());
  EXPECT_EQ(kinematics.orientation.w(), 1.0);
  EXPECT_EQ(kinematics.orientation.x(), 0.0);
  EXPECT_EQ(kinematics.orientation.y(), 0.0);
  EXPECT_EQ(kinematics.orientation.z(), 0.0);
  EXPECT_EQ(kinematics.linear_velocity, Eigen::Vector3d::Zero());
  EXPECT_EQ(kinematics.angular_velocity, Eigen::Vector3d::Zero());
  EXPECT_EQ(kinematics.linear_acceleration, Eigen::Vector3d::Zero());
  EXPECT_EQ(kinematics.angular_acceleration, Eigen::Vector3d::Zero());
}

// Test a custom constructor
TEST(State, Kinematics_constructor_custom) {
  // Create a custom instance of Kinematics with specific values
  Kinematics<float> kinematics;
  kinematics.position << 1.0f, 2.0f, 3.0f;
  kinematics.orientation = Eigen::Quaternion<float>(0.707f, 0.0f, 0.707f, 0.0f);
  kinematics.linear_velocity << 0.1f, 0.2f, 0.3f;
  kinematics.angular_velocity << 0.01f, 0.02f, 0.03f;
  kinematics.linear_acceleration << 0.01f, 0.02f, 0.03f;
  kinematics.angular_acceleration << 0.001f, 0.002f, 0.003f;

  // Check if the values were set correctly
  EXPECT_EQ(kinematics.position, Eigen::Vector3f(1.0f, 2.0f, 3.0f));
  // EXPECT_EQ(kinematics.orientation, Eigen::Quaternionf(0.707f, 0.0f, 0.707f, 0.0f));
  EXPECT_EQ(kinematics.orientation.w(), 0.707f);
  EXPECT_EQ(kinematics.orientation.x(), 0.0f);
  EXPECT_EQ(kinematics.orientation.y(), 0.707f);
  EXPECT_EQ(kinematics.orientation.z(), 0.0f);
  EXPECT_EQ(kinematics.linear_velocity, Eigen::Vector3f(0.1f, 0.2f, 0.3f));
  EXPECT_EQ(kinematics.angular_velocity, Eigen::Vector3f(0.01f, 0.02f, 0.03f));
  EXPECT_EQ(kinematics.linear_acceleration, Eigen::Vector3f(0.01f, 0.02f, 0.03f));
  EXPECT_EQ(kinematics.angular_acceleration, Eigen::Vector3f(0.001f, 0.002f, 0.003f));
}

// Test the default constructor
TEST(State, Dynamics_constructor_default) {
  Dynamics<double> dynamics;

  // Check if all fields are initialized to zero
  EXPECT_EQ(dynamics.force, Eigen::Vector3d::Zero());
  EXPECT_EQ(dynamics.torque, Eigen::Vector3d::Zero());
}

// Test a custom constructor
TEST(State, Dynamics_constructor_custom) {
  // Create a custom instance of Dynamics with specific values
  Dynamics<float> dynamics;
  dynamics.force << 1.0f, 2.0f, 3.0f;
  dynamics.torque << 0.1f, 0.2f, 0.3f;

  // Check if the values were set correctly
  EXPECT_EQ(dynamics.force, Eigen::Vector3f(1.0f, 2.0f, 3.0f));
  EXPECT_EQ(dynamics.torque, Eigen::Vector3f(0.1f, 0.2f, 0.3f));
}

// Test the default constructor
TEST(State, Actuators_constructor_default) {
  Actuators<double, 4> actuators;

  // Check if all fields are initialized to zero
  EXPECT_EQ(actuators.motor_angular_velocity, Eigen::Vector4d::Zero());
  EXPECT_EQ(actuators.motor_angular_acceleration, Eigen::Vector4d::Zero());
}

// Test a custom constructor
TEST(State, Actuators_constructor_custom) {
  // Create a custom instance of Actuators with specific values
  Actuators<float, 4> actuators;
  actuators.motor_angular_velocity << 0.1f, 0.2f, 0.3f, 0.4f;
  actuators.motor_angular_acceleration << 0.01f, 0.02f, 0.03f, 0.04f;

  // Check if the values were set correctly
  EXPECT_EQ(actuators.motor_angular_velocity, Eigen::Vector4f(0.1f, 0.2f, 0.3f, 0.4f));
  EXPECT_EQ(actuators.motor_angular_acceleration, Eigen::Vector4f(0.01f, 0.02f, 0.03f, 0.04f));
}

// Test the State struct with default template
TEST(State, State_default_template) {
  State<> state;

  // Check if all sub-structs are initialized to their default values (zero for
  // numeric types)
  EXPECT_EQ(state.kinematics.position, Eigen::Vector3d::Zero());
  // EXPECT_EQ(state.kinematics.orientation, Eigen::Quaterniond::Identity());
  EXPECT_EQ(state.kinematics.orientation.w(), 1.0);
  EXPECT_EQ(state.kinematics.orientation.x(), 0.0);
  EXPECT_EQ(state.kinematics.orientation.y(), 0.0);
  EXPECT_EQ(state.kinematics.orientation.z(), 0.0);
  EXPECT_EQ(state.kinematics.linear_velocity, Eigen::Vector3d::Zero());
  EXPECT_EQ(state.kinematics.angular_velocity, Eigen::Vector3d::Zero());
  EXPECT_EQ(state.kinematics.linear_acceleration, Eigen::Vector3d::Zero());
  EXPECT_EQ(state.kinematics.angular_acceleration, Eigen::Vector3d::Zero());

  EXPECT_EQ(state.dynamics.force, Eigen::Vector3d::Zero());
  EXPECT_EQ(state.dynamics.torque, Eigen::Vector3d::Zero());

  EXPECT_EQ(state.actuators.motor_angular_velocity, Eigen::Vector4d::Zero());
}

// Test the State struct with double precision
TEST(State, State_double_precision) {
  State<double, 4> state_double;

  // Check if all sub-structs are initialized to their default values (zero for
  // numeric types)
  EXPECT_EQ(state_double.kinematics.position, Eigen::Vector3d::Zero());
  // EXPECT_EQ(state_double.kinematics.orientation, Eigen::Quaterniond::Identity());
  EXPECT_EQ(state_double.kinematics.orientation.w(), 1.0);
  EXPECT_EQ(state_double.kinematics.orientation.x(), 0.0);
  EXPECT_EQ(state_double.kinematics.orientation.y(), 0.0);
  EXPECT_EQ(state_double.kinematics.orientation.z(), 0.0);
  EXPECT_EQ(state_double.kinematics.linear_velocity, Eigen::Vector3d::Zero());
  EXPECT_EQ(state_double.kinematics.angular_velocity, Eigen::Vector3d::Zero());
  EXPECT_EQ(state_double.kinematics.linear_acceleration, Eigen::Vector3d::Zero());
  EXPECT_EQ(state_double.kinematics.angular_acceleration, Eigen::Vector3d::Zero());

  EXPECT_EQ(state_double.dynamics.force, Eigen::Vector3d::Zero());
  EXPECT_EQ(state_double.dynamics.torque, Eigen::Vector3d::Zero());

  EXPECT_EQ(state_double.actuators.motor_angular_velocity, Eigen::Vector4d::Zero());
}

// Test the State struct with single precision
TEST(State, State_float_precision) {
  State<float, 4> state_float;

  // Check if all sub-structs are initialized to their default values (zero for
  // numeric types)
  EXPECT_EQ(state_float.kinematics.position, Eigen::Vector3f::Zero());
  // EXPECT_EQ(state_float.kinematics.orientation, Eigen::Quaternionf::Identity());
  EXPECT_EQ(state_float.kinematics.orientation.w(), 1.0f);
  EXPECT_EQ(state_float.kinematics.orientation.x(), 0.0f);
  EXPECT_EQ(state_float.kinematics.orientation.y(), 0.0f);
  EXPECT_EQ(state_float.kinematics.orientation.z(), 0.0f);
  EXPECT_EQ(state_float.kinematics.linear_velocity, Eigen::Vector3f::Zero());
  EXPECT_EQ(state_float.kinematics.angular_velocity, Eigen::Vector3f::Zero());
  EXPECT_EQ(state_float.kinematics.linear_acceleration, Eigen::Vector3f::Zero());
  EXPECT_EQ(state_float.kinematics.angular_acceleration, Eigen::Vector3f::Zero());

  EXPECT_EQ(state_float.dynamics.force, Eigen::Vector3f::Zero());
  EXPECT_EQ(state_float.dynamics.torque, Eigen::Vector3f::Zero());

  EXPECT_EQ(state_float.actuators.motor_angular_velocity, Eigen::Vector4f::Zero());
  EXPECT_EQ(state_float.actuators.motor_angular_acceleration, Eigen::Vector4f::Zero());
}

TEST(State, State_string) {
  // Test quaternion_to_Euler
  Eigen::Quaterniond quat = Eigen::Quaterniond::Identity();
  double roll, pitch, yaw;
  multirotor::state::internal::quaternion_to_Euler<double>(quat, roll, pitch, yaw);
  EXPECT_EQ(roll, 0.0);
  EXPECT_EQ(pitch, 0.0);
  EXPECT_EQ(yaw, 0.0);
  EXPECT_NO_THROW(multirotor::state::internal::quaternion_to_Euler<double>(quat, roll, pitch, yaw));

  double a = 1.0;
  double b = 2.0;
  double c = 3.0;
  double d = 4.0;
  double e = 5.0;
  double f = 6.0;
  double g = 7.0;
  double h = 8.0;
  double i = 9.0;
  double j = 10.0;
  double k = 11.0;
  double l = 12.0;
  double m = 13.0;
  double n = 14.0;
  double o = 15.0;
  double p = 16.0;
  double q = 17.0;
  double r = 18.0;
  double s = 19.0;
  double t = 20.0;
  double u = 21.0;
  double v = 22.0;
  double w = 23.0;
  double x = 24.0;
  double y = 25.0;
  double z = 26.0;
  State<> state;
  state.kinematics.position << a, b, c;
  state.kinematics.orientation = Eigen::Quaterniond::Identity();
  state.kinematics.linear_velocity << d, e, f;
  state.kinematics.angular_velocity << g, h, i;
  state.kinematics.linear_acceleration << j, k, l;
  state.kinematics.angular_acceleration << m, n, o;
  state.dynamics.force << p, q, r;
  state.dynamics.torque << s, t, u;
  state.actuators.motor_angular_velocity << v, w, x, y;
  state.actuators.motor_angular_acceleration << z, a, b, c;

  std::stringstream state_str;
  state_str << state;

  std::stringstream state_str_expected;
  state_str_expected << a << "," << b << "," << c << "," << 1.0 << "," << 0.0 << "," << 0.0 << ","
                     << 0.0 << "," << 0.0 << "," << 0.0 << "," << 0.0 << "," << d << "," << e << ","
                     << f << "," << g << "," << h << "," << i << "," << j << "," << k << "," << l
                     << "," << m << "," << n << "," << o << "," << p << "," << q << "," << r << ","
                     << s << "," << t << "," << u << "," << v << "," << w << "," << x << "," << y
                     << "," << z << "," << a << "," << b << "," << c;
  EXPECT_EQ(state_str.str(), state_str_expected.str());

  EXPECT_NO_THROW(print_state(state));
  EXPECT_NO_THROW(std::cout << state << std::endl);
}

int main(int argc, char *argv[]) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
