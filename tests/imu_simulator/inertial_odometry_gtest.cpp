/*!*******************************************************************************************
 *  \file       inertial_odometry_gtest.hpp
 *  \brief      Inertial Odometry unit tests
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
#include <chrono>
#include <memory>

#include "imu_simulator/inertial_odometry.hpp"

template <typename P = double>
using InertialOdometryParams = imu::InertialOdometryParams<P>;
template <typename P = double>
using InertialOdometry = imu::InertialOdometry<P>;

TEST(InertialOdometryTest, constructors) {
  EXPECT_NO_THROW(InertialOdometry<> inertial_odometry = InertialOdometry<>());
}

// Test the default constructor
TEST(InertialOdometryTest, constructor2) {
  double alpha = 0.1;
  EXPECT_NO_THROW(InertialOdometry<double> inertial_odometry = InertialOdometry<double>(alpha));

  InertialOdometry<double> inertial_odometry = InertialOdometry<double>(alpha);

  // Test public methods
  EXPECT_NO_THROW(inertial_odometry.reset());

  double dt               = 0.1;
  Eigen::Vector3d vector1 = Eigen::Vector3d::Random();
  Eigen::Vector3d vector2 = Eigen::Vector3d::Random();
  EXPECT_NO_THROW(inertial_odometry.update(vector1, vector2, dt));

  Eigen::Vector3d position;
  Eigen::Quaterniond orientation;
  Eigen::Vector3d linear_velocity;
  Eigen::Vector3d angular_velocity;
  Eigen::Vector3d linear_acceleration;
  EXPECT_NO_THROW(inertial_odometry.get_measurement(position, orientation, linear_velocity,
                                                    angular_velocity, linear_acceleration));
  EXPECT_NO_THROW(position = inertial_odometry.get_position());
  EXPECT_NO_THROW(orientation = inertial_odometry.get_orientation());
  EXPECT_NO_THROW(linear_velocity = inertial_odometry.get_linear_velocity());
  EXPECT_NO_THROW(angular_velocity = inertial_odometry.get_angular_velocity());
  EXPECT_NO_THROW(linear_acceleration = inertial_odometry.get_linear_acceleration());
}

TEST(InertialOdometryTest, constructor_params) {
  double alpha = 0.1;
  InertialOdometryParams<double> params;
  params.alpha = alpha;

  EXPECT_NO_THROW(InertialOdometry<double> inertial_odometry = InertialOdometry<double>(params));
}

// Test the acceleration integration of the inertial odometry
TEST(InertialOdometryTest, acceleration_integration) {
  double alpha                               = 0.1;
  Eigen::Vector3d gravity                    = Eigen::Vector3d(0, 0, -9.81);
  InertialOdometry<double> inertial_odometry = InertialOdometry<double>(alpha);

  // Init variables
  double dt                     = 0.001;
  Eigen::Vector3d gyroscope     = Eigen::Vector3d::Zero();
  Eigen::Vector3d accelerometer = Eigen::Vector3d::Zero();

  Eigen::Vector3d position            = Eigen::Vector3d::Zero();
  Eigen::Quaterniond orientation      = Eigen::Quaterniond::Identity();
  Eigen::Vector3d linear_velocity     = Eigen::Vector3d::Zero();
  Eigen::Vector3d angular_velocity    = Eigen::Vector3d::Zero();
  Eigen::Vector3d linear_acceleration = Eigen::Vector3d::Zero();

  int num_iterations = 100;

  // Test acceleration integration
  double acceleration_magitude = 1.0;  // m/s^2

  // Acceleration in x
  inertial_odometry.reset();
  accelerometer = Eigen::Vector3d::UnitX() * acceleration_magitude - gravity;

  for (int i = 0; i < num_iterations; i++) {
    inertial_odometry.update(gyroscope, accelerometer, dt);
  }
  inertial_odometry.get_measurement(position, orientation, linear_velocity, angular_velocity,
                                    linear_acceleration);

  EXPECT_GT(linear_acceleration.x(), 0.0);
  EXPECT_EQ(linear_acceleration.y(), 0.0);
  EXPECT_EQ(linear_acceleration.z(), 0.0);
  EXPECT_GT(linear_velocity.x(), 0.0);
  EXPECT_EQ(linear_velocity.y(), 0.0);
  EXPECT_EQ(linear_velocity.z(), 0.0);
  EXPECT_GT(position.x(), 0.0);
  EXPECT_EQ(position.y(), 0.0);
  EXPECT_EQ(position.z(), 0.0);

  // Desacceleration in x
  inertial_odometry.reset();
  accelerometer = Eigen::Vector3d::UnitX() * -acceleration_magitude - gravity;

  for (int i = 0; i < num_iterations; i++) {
    inertial_odometry.update(gyroscope, accelerometer, dt);
  }
  inertial_odometry.get_measurement(position, orientation, linear_velocity, angular_velocity,
                                    linear_acceleration);

  EXPECT_LT(linear_acceleration.x(), 0.0);
  EXPECT_EQ(linear_acceleration.y(), 0.0);
  EXPECT_EQ(linear_acceleration.z(), 0.0);
  EXPECT_LT(linear_velocity.x(), 0.0);
  EXPECT_EQ(linear_velocity.y(), 0.0);
  EXPECT_EQ(linear_velocity.z(), 0.0);
  EXPECT_LT(position.x(), 0.0);
  EXPECT_EQ(position.y(), 0.0);
  EXPECT_EQ(position.z(), 0.0);

  // Acceleration in y
  inertial_odometry.reset();
  accelerometer = Eigen::Vector3d::UnitY() * acceleration_magitude - gravity;

  for (int i = 0; i < num_iterations; i++) {
    inertial_odometry.update(gyroscope, accelerometer, dt);
  }
  inertial_odometry.get_measurement(position, orientation, linear_velocity, angular_velocity,
                                    linear_acceleration);

  EXPECT_EQ(linear_acceleration.x(), 0.0);
  EXPECT_GT(linear_acceleration.y(), 0.0);
  EXPECT_EQ(linear_acceleration.z(), 0.0);
  EXPECT_EQ(linear_velocity.x(), 0.0);
  EXPECT_GT(linear_velocity.y(), 0.0);
  EXPECT_EQ(linear_velocity.z(), 0.0);
  EXPECT_EQ(position.x(), 0.0);
  EXPECT_GT(position.y(), 0.0);
  EXPECT_EQ(position.z(), 0.0);

  // Desacceleration in y
  inertial_odometry.reset();
  accelerometer = Eigen::Vector3d::UnitY() * -acceleration_magitude - gravity;

  for (int i = 0; i < num_iterations; i++) {
    inertial_odometry.update(gyroscope, accelerometer, dt);
  }
  inertial_odometry.get_measurement(position, orientation, linear_velocity, angular_velocity,
                                    linear_acceleration);

  EXPECT_EQ(linear_acceleration.x(), 0.0);
  EXPECT_LT(linear_acceleration.y(), 0.0);
  EXPECT_EQ(linear_acceleration.z(), 0.0);
  EXPECT_EQ(linear_velocity.x(), 0.0);
  EXPECT_LT(linear_velocity.y(), 0.0);
  EXPECT_EQ(linear_velocity.z(), 0.0);
  EXPECT_EQ(position.x(), 0.0);
  EXPECT_LT(position.y(), 0.0);
  EXPECT_EQ(position.z(), 0.0);

  // Acceleration in z
  inertial_odometry.reset();
  accelerometer = Eigen::Vector3d::UnitZ() * acceleration_magitude - gravity;

  for (int i = 0; i < num_iterations; i++) {
    inertial_odometry.update(gyroscope, accelerometer, dt);
  }
  inertial_odometry.get_measurement(position, orientation, linear_velocity, angular_velocity,
                                    linear_acceleration);

  EXPECT_EQ(linear_acceleration.x(), 0.0);
  EXPECT_EQ(linear_acceleration.y(), 0.0);
  EXPECT_GT(linear_acceleration.z(), 0.0);
  EXPECT_EQ(linear_velocity.x(), 0.0);
  EXPECT_EQ(linear_velocity.y(), 0.0);
  EXPECT_GT(linear_velocity.z(), 0.0);
  EXPECT_EQ(position.x(), 0.0);
  EXPECT_EQ(position.y(), 0.0);
  EXPECT_GT(position.z(), 0.0);

  // Desacceleration in z
  inertial_odometry.reset();
  accelerometer = Eigen::Vector3d::UnitZ() * -acceleration_magitude - gravity;

  for (int i = 0; i < num_iterations; i++) {
    inertial_odometry.update(gyroscope, accelerometer, dt);
  }
  inertial_odometry.get_measurement(position, orientation, linear_velocity, angular_velocity,
                                    linear_acceleration);

  EXPECT_EQ(linear_acceleration.x(), 0.0);
  EXPECT_EQ(linear_acceleration.y(), 0.0);
  EXPECT_LT(linear_acceleration.z(), 0.0);
  EXPECT_EQ(linear_velocity.x(), 0.0);
  EXPECT_EQ(linear_velocity.y(), 0.0);
  EXPECT_LT(linear_velocity.z(), 0.0);
  EXPECT_EQ(position.x(), 0.0);
  EXPECT_EQ(position.y(), 0.0);
  EXPECT_LT(position.z(), 0.0);
}

void quaternion_to_euler(const Eigen::Quaterniond quaternion,
                         double& roll,
                         double& pitch,
                         double& yaw) {
  // Extract individual components of the quaternion
  double qw = quaternion.w();
  double qx = quaternion.x();
  double qy = quaternion.y();
  double qz = quaternion.z();

  // Compute roll (rotation around x-axis)
  double sinr_cosp = 2.0 * (qw * qx + qy * qz);
  double cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy);
  roll             = std::atan2(sinr_cosp, cosr_cosp);

  // Compute pitch (rotation around y-axis)
  double sinp = 2.0 * (qw * qy - qz * qx);
  if (std::abs(sinp) >= 1)
    pitch = std::copysign(M_PI / 2, sinp);  // Use 90 degrees if out of range
  else
    pitch = std::asin(sinp);

  // Compute yaw (rotation around z-axis)
  double siny_cosp = 2.0 * (qw * qz + qx * qy);
  double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
  yaw              = std::atan2(siny_cosp, cosy_cosp);
}

// Test the gyroscope integration of the inertial odometry
TEST(InertialOdometryTest, gyroscope_integration) {
  double alpha                               = 0.1;
  InertialOdometry<double> inertial_odometry = InertialOdometry<double>(alpha);

  // Init variables
  double dt                     = 0.001;
  Eigen::Vector3d gyroscope     = Eigen::Vector3d::Zero();
  Eigen::Vector3d accelerometer = Eigen::Vector3d::Zero();

  Eigen::Vector3d position            = Eigen::Vector3d::Zero();
  Eigen::Quaterniond orientation      = Eigen::Quaterniond::Identity();
  Eigen::Vector3d linear_velocity     = Eigen::Vector3d::Zero();
  Eigen::Vector3d angular_velocity    = Eigen::Vector3d::Zero();
  Eigen::Vector3d linear_acceleration = Eigen::Vector3d::Zero();
  double roll, pitch, yaw;

  int num_iterations = 100;

  // Test gyroscope integration
  double gyroscope_magitude = 0.1;  // rad/s

  // Rotation around x
  inertial_odometry.reset();
  gyroscope = Eigen::Vector3d::UnitX() * gyroscope_magitude;

  for (int i = 0; i < num_iterations; i++) {
    inertial_odometry.update(gyroscope, accelerometer, dt);
  }
  inertial_odometry.get_measurement(position, orientation, linear_velocity, angular_velocity,
                                    linear_acceleration);

  EXPECT_GE(angular_velocity.x(), 0.0);
  EXPECT_EQ(angular_velocity.y(), 0.0);
  EXPECT_EQ(angular_velocity.z(), 0.0);
  quaternion_to_euler(orientation, roll, pitch, yaw);
  EXPECT_GT(roll, 0.0);
  EXPECT_EQ(pitch, 0.0);
  EXPECT_EQ(yaw, 0.0);

  // Rotation around -x
  inertial_odometry.reset();
  gyroscope = Eigen::Vector3d::UnitX() * -gyroscope_magitude;

  for (int i = 0; i < num_iterations; i++) {
    inertial_odometry.update(gyroscope, accelerometer, dt);
  }
  inertial_odometry.get_measurement(position, orientation, linear_velocity, angular_velocity,
                                    linear_acceleration);

  EXPECT_LE(angular_velocity.x(), 0.0);
  EXPECT_EQ(angular_velocity.y(), 0.0);
  EXPECT_EQ(angular_velocity.z(), 0.0);
  quaternion_to_euler(orientation, roll, pitch, yaw);
  EXPECT_LT(roll, 0.0);
  EXPECT_EQ(pitch, 0.0);
  EXPECT_EQ(yaw, 0.0);

  // Rotation around y
  inertial_odometry.reset();
  gyroscope = Eigen::Vector3d::UnitY() * gyroscope_magitude;

  for (int i = 0; i < num_iterations; i++) {
    inertial_odometry.update(gyroscope, accelerometer, dt);
  }
  inertial_odometry.get_measurement(position, orientation, linear_velocity, angular_velocity,
                                    linear_acceleration);

  EXPECT_EQ(angular_velocity.x(), 0.0);
  EXPECT_GE(angular_velocity.y(), 0.0);
  EXPECT_EQ(angular_velocity.z(), 0.0);
  quaternion_to_euler(orientation, roll, pitch, yaw);
  EXPECT_EQ(roll, 0.0);
  EXPECT_GT(pitch, 0.0);
  EXPECT_EQ(yaw, 0.0);

  // Rotation around -y
  inertial_odometry.reset();
  gyroscope = Eigen::Vector3d::UnitY() * -gyroscope_magitude;

  for (int i = 0; i < num_iterations; i++) {
    inertial_odometry.update(gyroscope, accelerometer, dt);
  }
  inertial_odometry.get_measurement(position, orientation, linear_velocity, angular_velocity,
                                    linear_acceleration);

  EXPECT_EQ(angular_velocity.x(), 0.0);
  EXPECT_LE(angular_velocity.y(), 0.0);
  EXPECT_EQ(angular_velocity.z(), 0.0);
  quaternion_to_euler(orientation, roll, pitch, yaw);
  EXPECT_EQ(roll, 0.0);
  EXPECT_LT(pitch, 0.0);
  EXPECT_EQ(yaw, 0.0);

  // Rotation around z
  inertial_odometry.reset();
  gyroscope = Eigen::Vector3d::UnitZ() * gyroscope_magitude;

  for (int i = 0; i < num_iterations; i++) {
    inertial_odometry.update(gyroscope, accelerometer, dt);
  }
  inertial_odometry.get_measurement(position, orientation, linear_velocity, angular_velocity,
                                    linear_acceleration);

  EXPECT_EQ(angular_velocity.x(), 0.0);
  EXPECT_EQ(angular_velocity.y(), 0.0);
  EXPECT_GE(angular_velocity.z(), 0.0);
  quaternion_to_euler(orientation, roll, pitch, yaw);
  EXPECT_EQ(roll, 0.0);
  EXPECT_EQ(pitch, 0.0);
  EXPECT_GT(yaw, 0.0);

  // Rotation around -z
  inertial_odometry.reset();
  gyroscope = Eigen::Vector3d::UnitZ() * -gyroscope_magitude;

  for (int i = 0; i < num_iterations; i++) {
    inertial_odometry.update(gyroscope, accelerometer, dt);
  }
  inertial_odometry.get_measurement(position, orientation, linear_velocity, angular_velocity,
                                    linear_acceleration);

  EXPECT_EQ(angular_velocity.x(), 0.0);
  EXPECT_EQ(angular_velocity.y(), 0.0);
  EXPECT_LE(angular_velocity.z(), 0.0);
  quaternion_to_euler(orientation, roll, pitch, yaw);
  EXPECT_EQ(roll, 0.0);
  EXPECT_EQ(pitch, 0.0);
  EXPECT_LT(yaw, 0.0);
}

TEST(InertialOdometryTest, print_methods) {
  InertialOdometry<> inertial_odometry = InertialOdometry<>();
  EXPECT_NO_THROW(print_state(std::cout, inertial_odometry));
  EXPECT_NO_THROW(std::cout << inertial_odometry << std::endl);
}

Eigen::Quaterniond euler_to_quaternion(double roll, double pitch, double yaw) {
  // Calculate half angles
  double roll_half  = roll * 0.5;
  double pitch_half = pitch * 0.5;
  double yaw_half   = yaw * 0.5;

  // Calculate the sine and cosine of the half angles
  double sr = sin(roll_half);
  double cr = cos(roll_half);
  double sp = sin(pitch_half);
  double cp = cos(pitch_half);
  double sy = sin(yaw_half);
  double cy = cos(yaw_half);

  // Calculate the quaternion components
  double w = cr * cp * cy + sr * sp * sy;
  double x = sr * cp * cy - cr * sp * sy;
  double y = cr * sp * cy + sr * cp * sy;
  double z = cr * cp * sy - sr * sp * cy;

  // Create the Quaternion object
  return Eigen::Quaterniond(w, x, y, z).normalized();
}

TEST(InertialOdometryTest, initial_orientation) {
  double alpha            = 0.1;
  Eigen::Vector3d gravity = Eigen::Vector3d(0, 0, -9.81);

  // Init variables
  double epsilon                = 0.001;
  double dt                     = 0.001;
  Eigen::Vector3d gyroscope     = Eigen::Vector3d::Zero();
  Eigen::Vector3d accelerometer = Eigen::Vector3d::Zero();

  Eigen::Vector3d position            = Eigen::Vector3d::Zero();
  double roll                         = 0.0;
  double pitch                        = 0.0;
  double yaw                          = M_PI_2;
  Eigen::Quaterniond orientation      = euler_to_quaternion(roll, pitch, yaw);
  Eigen::Vector3d linear_velocity     = Eigen::Vector3d::Zero();
  Eigen::Vector3d angular_velocity    = Eigen::Vector3d::Zero();
  Eigen::Vector3d linear_acceleration = Eigen::Vector3d::Zero();

  InertialOdometryParams<double> params;
  params.alpha                               = alpha;
  params.initial_world_orientation           = orientation;
  params.initial_world_position              = position;
  params.gravity                             = gravity;
  InertialOdometry<double> inertial_odometry = InertialOdometry<double>(params);

  int num_iterations = 100;

  // Test acceleration integration
  double acceleration_magitude = 1.0;  // m/s^2

  // World accelerations
  Eigen::Vector3d accel_world_x = Eigen::Vector3d::UnitX() * acceleration_magitude;
  Eigen::Vector3d accel_world_y = Eigen::Vector3d::UnitY() * acceleration_magitude;
  Eigen::Vector3d accel_world_z = Eigen::Vector3d::UnitZ() * acceleration_magitude;

  // Baselink accelerations
  Eigen::Vector3d base_link_x_p = orientation.inverse() * accel_world_x - gravity;
  Eigen::Vector3d base_link_x_n = orientation.inverse() * -accel_world_x - gravity;
  Eigen::Vector3d base_link_y_p = orientation.inverse() * accel_world_y - gravity;
  Eigen::Vector3d base_link_y_n = orientation.inverse() * -accel_world_y - gravity;
  Eigen::Vector3d base_link_z_p = orientation.inverse() * accel_world_z - gravity;
  Eigen::Vector3d base_link_z_n = orientation.inverse() * -accel_world_z - gravity;

  // Positive acceleration in x world frame
  inertial_odometry.reset();
  accelerometer = base_link_x_p;
  for (int i = 0; i < num_iterations; i++) {
    inertial_odometry.update(gyroscope, accelerometer, dt);
  }
  inertial_odometry.get_measurement(position, orientation, linear_velocity, angular_velocity,
                                    linear_acceleration);

  EXPECT_GT(linear_acceleration.x(), 0.0);
  EXPECT_NEAR(linear_acceleration.y(), 0.0, epsilon);
  EXPECT_NEAR(linear_acceleration.z(), 0.0, epsilon);
  EXPECT_GT(linear_velocity.x(), 0.0);
  EXPECT_NEAR(linear_velocity.y(), 0.0, epsilon);
  EXPECT_NEAR(linear_velocity.z(), 0.0, epsilon);
  EXPECT_GT(position.x(), 0.0);
  EXPECT_NEAR(position.y(), 0.0, epsilon);
  EXPECT_NEAR(position.z(), 0.0, epsilon);

  // Negative acceleration in x world frame
  inertial_odometry.reset();
  accelerometer = base_link_x_n;
  for (int i = 0; i < num_iterations; i++) {
    inertial_odometry.update(gyroscope, accelerometer, dt);
  }
  inertial_odometry.get_measurement(position, orientation, linear_velocity, angular_velocity,
                                    linear_acceleration);

  EXPECT_LT(linear_acceleration.x(), 0.0);
  EXPECT_NEAR(linear_acceleration.y(), 0.0, epsilon);
  EXPECT_NEAR(linear_acceleration.z(), 0.0, epsilon);
  EXPECT_LT(linear_velocity.x(), 0.0);
  EXPECT_NEAR(linear_velocity.y(), 0.0, epsilon);
  EXPECT_NEAR(linear_velocity.z(), 0.0, epsilon);
  EXPECT_LT(position.x(), 0.0);
  EXPECT_NEAR(position.y(), 0.0, epsilon);
  EXPECT_NEAR(position.z(), 0.0, epsilon);

  // Positive acceleration in y world frame
  inertial_odometry.reset();
  accelerometer = base_link_y_p;
  for (int i = 0; i < num_iterations; i++) {
    inertial_odometry.update(gyroscope, accelerometer, dt);
  }
  inertial_odometry.get_measurement(position, orientation, linear_velocity, angular_velocity,
                                    linear_acceleration);

  EXPECT_NEAR(linear_acceleration.x(), 0.0, epsilon);
  EXPECT_GT(linear_acceleration.y(), 0.0);
  EXPECT_NEAR(linear_acceleration.z(), 0.0, epsilon);
  EXPECT_NEAR(linear_velocity.x(), 0.0, epsilon);
  EXPECT_GT(linear_velocity.y(), 0.0);
  EXPECT_NEAR(linear_velocity.z(), 0.0, epsilon);
  EXPECT_NEAR(position.x(), 0.0, epsilon);
  EXPECT_GT(position.y(), 0.0);
  EXPECT_NEAR(position.z(), 0.0, epsilon);

  // Negative acceleration in y world frame
  inertial_odometry.reset();
  accelerometer = base_link_y_n;
  for (int i = 0; i < num_iterations; i++) {
    inertial_odometry.update(gyroscope, accelerometer, dt);
  }
  inertial_odometry.get_measurement(position, orientation, linear_velocity, angular_velocity,
                                    linear_acceleration);

  EXPECT_NEAR(linear_acceleration.x(), 0.0, epsilon);
  EXPECT_LT(linear_acceleration.y(), 0.0);
  EXPECT_NEAR(linear_acceleration.z(), 0.0, epsilon);
  EXPECT_NEAR(linear_velocity.x(), 0.0, epsilon);
  EXPECT_LT(linear_velocity.y(), 0.0);
  EXPECT_NEAR(linear_velocity.z(), 0.0, epsilon);
  EXPECT_NEAR(position.x(), 0.0, epsilon);
  EXPECT_LT(position.y(), 0.0);
  EXPECT_NEAR(position.z(), 0.0, epsilon);

  // Positive acceleration in z world frame
  inertial_odometry.reset();
  accelerometer = base_link_z_p;
  for (int i = 0; i < num_iterations; i++) {
    inertial_odometry.update(gyroscope, accelerometer, dt);
  }
  inertial_odometry.get_measurement(position, orientation, linear_velocity, angular_velocity,
                                    linear_acceleration);

  EXPECT_NEAR(linear_acceleration.x(), 0.0, epsilon);
  EXPECT_NEAR(linear_acceleration.y(), 0.0, epsilon);
  EXPECT_GT(linear_acceleration.z(), 0.0);
  EXPECT_NEAR(linear_velocity.x(), 0.0, epsilon);
  EXPECT_NEAR(linear_velocity.y(), 0.0, epsilon);
  EXPECT_GT(linear_velocity.z(), 0.0);
  EXPECT_NEAR(position.x(), 0.0, epsilon);
  EXPECT_NEAR(position.y(), 0.0, epsilon);
  EXPECT_GT(position.z(), 0.0);

  // Negative acceleration in z world frame
  inertial_odometry.reset();
  accelerometer = base_link_z_n;
  for (int i = 0; i < num_iterations; i++) {
    inertial_odometry.update(gyroscope, accelerometer, dt);
  }
  inertial_odometry.get_measurement(position, orientation, linear_velocity, angular_velocity,
                                    linear_acceleration);

  EXPECT_NEAR(linear_acceleration.x(), 0.0, epsilon);
  EXPECT_NEAR(linear_acceleration.y(), 0.0, epsilon);
  EXPECT_LT(linear_acceleration.z(), 0.0);
  EXPECT_NEAR(linear_velocity.x(), 0.0, epsilon);
  EXPECT_NEAR(linear_velocity.y(), 0.0, epsilon);
  EXPECT_LT(linear_velocity.z(), 0.0);
  EXPECT_NEAR(position.x(), 0.0, epsilon);
  EXPECT_NEAR(position.y(), 0.0, epsilon);
  EXPECT_LT(position.z(), 0.0);
}

int main(int argc, char* argv[]) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
