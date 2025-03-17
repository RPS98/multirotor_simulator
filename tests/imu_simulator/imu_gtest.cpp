/*!*******************************************************************************************
 *  \file       imu_gtest.hpp
 *  \brief      Inertial Measurement Unit unit tests
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

#include "imu_simulator/imu.hpp"

template <typename P = double>
using IMUParams = imu::IMUParams<P>;
template <typename P = double>
using IMU = imu::IMU<P>;

// Class child of IMU to access protected methods and convert to public
template <typename T>
class PublicIMU : public imu::IMU<T> {
public:
  // Inherit the constructor from the base class
  using IMU<T>::IMU;

  // Make the protected functions public in the child class
  using IMU<T>::set_bias_standard_normal_distribution;
  using IMU<T>::process_bias;
  using IMU<T>::process_measurement;

  // Make the protected variables public in the child class
  using IMU<T>::random_number_generator_;
  using IMU<T>::standard_normal_distribution_;
  using IMU<T>::gyro_noise_var_;
  using IMU<T>::accel_noise_var_;
  using IMU<T>::gyro_bias_noise_autocorr_time_;
  using IMU<T>::accel_bias_noise_autocorr_time_;
  using IMU<T>::imu_orientation_;
  using IMU<T>::imu_orientation_inverse_;
  using IMU<T>::gyro_bias_;
  using IMU<T>::accel_bias_;
  using IMU<T>::gyro_;
  using IMU<T>::accel_;
};

// Test the default constructor
TEST(IMUTest, constructor_default) { EXPECT_NO_THROW(IMU<> _imu = IMU<>()); }

TEST(IMUTest, constructor2) {
  double gyro_noise_var                 = 0.003;
  double accel_noise_var                = 0.005;
  double gyro_bias_noise_autocorr_time  = 1.0e-7;
  double accel_bias_noise_autocorr_time = 1.0e-7;
  Eigen::Quaterniond imu_orientation    = Eigen::Quaterniond::Identity();
  Eigen::Vector3d gravity               = Eigen::Vector3d(0, 0, -9.81);

  IMU<double> imu = IMU<double>(gyro_noise_var, accel_noise_var, gyro_bias_noise_autocorr_time,
                                accel_bias_noise_autocorr_time, imu_orientation, gravity);
}

TEST(IMUTest, constructor_params) {
  double gyro_noise_var                 = 0.003;
  double accel_noise_var                = 0.005;
  double gyro_bias_noise_autocorr_time  = 1.0e-7;
  double accel_bias_noise_autocorr_time = 1.0e-7;
  Eigen::Quaterniond imu_orientation    = Eigen::Quaterniond::Identity();

  IMUParams<double> imu_params;
  imu_params.gyro_noise_var                 = gyro_noise_var;
  imu_params.accel_noise_var                = accel_noise_var;
  imu_params.gyro_bias_noise_autocorr_time  = gyro_bias_noise_autocorr_time;
  imu_params.accel_bias_noise_autocorr_time = accel_bias_noise_autocorr_time;
  imu_params.imu_orientation                = imu_orientation;

  EXPECT_NO_THROW(IMU<double> imu = IMU<double>(imu_params));
}

TEST(IMUTest, public_methods) {
  double gyro_noise_var                 = 0.003;
  double accel_noise_var                = 0.005;
  double gyro_bias_noise_autocorr_time  = 1.0e-7;
  double accel_bias_noise_autocorr_time = 1.0e-7;
  Eigen::Quaterniond imu_orientation    = Eigen::Quaterniond::Identity();

  PublicIMU<double> imu =
      PublicIMU<double>(gyro_noise_var, accel_noise_var, gyro_bias_noise_autocorr_time,
                        accel_bias_noise_autocorr_time, imu_orientation);

  Eigen::Vector3d vector  = Eigen::Vector3d::Ones();
  Eigen::Vector3d vector1 = Eigen::Vector3d::Ones();
  Eigen::Vector3d vector2 = Eigen::Vector3d::Ones();

  EXPECT_NO_THROW(imu.update(0.01, vector, vector));
  EXPECT_NO_THROW(imu.get_measurement(vector1, vector2));
  EXPECT_NO_THROW(imu.reset());

  // Setters
  EXPECT_NO_THROW(imu.set_random_seed(1));
  EXPECT_NO_THROW(imu.set_noise_variances(0.01, 0.01));
  EXPECT_EQ(imu.gyro_noise_var_, 0.01);
  EXPECT_EQ(imu.accel_noise_var_, 0.01);
  EXPECT_NO_THROW(imu.set_bias_autocorrelation_time(0.01, 0.01));
  EXPECT_EQ(imu.gyro_bias_noise_autocorr_time_, 0.01);
  EXPECT_EQ(imu.accel_bias_noise_autocorr_time_, 0.01);
  EXPECT_NO_THROW(imu.set_bias(vector, vector, 0.02, 0.02));
  EXPECT_EQ(imu.gyro_bias_, vector);
  EXPECT_EQ(imu.accel_bias_, vector);
  EXPECT_EQ(imu.gyro_bias_noise_autocorr_time_, 0.02);
  EXPECT_EQ(imu.accel_bias_noise_autocorr_time_, 0.02);
  imu_orientation = Eigen::Quaterniond(0.5, 0.5, 0.5, 0.5);
  imu_orientation.normalize();
  EXPECT_NO_THROW(imu.set_orientation(imu_orientation));
  // EXPECT_EQ(imu.imu_orientation_, imu_orientation);
  EXPECT_EQ(imu.imu_orientation_.w(), imu_orientation.w());
  EXPECT_EQ(imu.imu_orientation_.x(), imu_orientation.x());
  EXPECT_EQ(imu.imu_orientation_.y(), imu_orientation.y());
  EXPECT_EQ(imu.imu_orientation_.z(), imu_orientation.z());
  // EXPECT_EQ(imu.imu_orientation_inverse, imu_orientation.inverse());
  EXPECT_EQ(imu.imu_orientation_inverse_.w(), imu_orientation.inverse().w());
  EXPECT_EQ(imu.imu_orientation_inverse_.x(), imu_orientation.inverse().x());
  EXPECT_EQ(imu.imu_orientation_inverse_.y(), imu_orientation.inverse().y());
  EXPECT_EQ(imu.imu_orientation_inverse_.z(), imu_orientation.inverse().z());

  // Getters
  EXPECT_NO_THROW(imu.get_bias(vector1, vector2));
  EXPECT_EQ(imu.gyro_bias_, vector1);
  EXPECT_EQ(imu.accel_bias_, vector2);
  EXPECT_NO_THROW(imu.get_variances(gyro_noise_var, accel_noise_var));
  EXPECT_EQ(imu.gyro_noise_var_, gyro_noise_var);
  EXPECT_EQ(imu.accel_noise_var_, accel_noise_var);
  EXPECT_NO_THROW(imu.get_measurement_gyro(vector1));
  EXPECT_EQ(imu.gyro_, vector1);
  EXPECT_NO_THROW(vector1 = imu.get_measurement_gyro());
  EXPECT_EQ(imu.gyro_, vector1);
  EXPECT_NO_THROW(imu.get_measurement_accel(vector1));
  EXPECT_EQ(imu.accel_, vector1);
  EXPECT_NO_THROW(vector1 = imu.get_measurement_gyro());
  EXPECT_EQ(imu.gyro_, vector1);
}

TEST(IMUTest, process_measurement_no_noise) {
  // Remove noise from the measurements
  double gyro_noise_var                 = 0.0;
  double accel_noise_var                = 0.0;
  double gyro_bias_noise_autocorr_time  = 0.0;
  double accel_bias_noise_autocorr_time = 0.0;
  Eigen::Quaterniond imu_orientation    = Eigen::Quaterniond::Identity();

  PublicIMU<double> imu =
      PublicIMU<double>(gyro_noise_var, accel_noise_var, gyro_bias_noise_autocorr_time,
                        accel_bias_noise_autocorr_time, imu_orientation);

  Eigen::Vector3d specific_force           = Eigen::Vector3d::Zero();
  Eigen::Vector3d vehicle_angular_velocity = Eigen::Vector3d::Zero();
  Eigen::Vector3d gyroscope                = Eigen::Vector3d::Zero();
  Eigen::Vector3d accelerometer            = Eigen::Vector3d::Zero();

  EXPECT_NO_THROW(imu.process_measurement(specific_force, vehicle_angular_velocity));

  // Test accelerometer measurement
  specific_force = Eigen::Vector3d::Zero();
  accelerometer  = Eigen::Vector3d::Zero();

  // If force is zero, then the measurement should be zero
  imu.process_measurement(specific_force, vehicle_angular_velocity);
  imu.get_measurement(gyroscope, accelerometer);
  EXPECT_EQ(accelerometer(0), 0.0);
  EXPECT_EQ(accelerometer(1), 0.0);
  EXPECT_EQ(accelerometer(2), 0.0);

  // Force and measurement sould have the same direction

  // If force is in x direction, then the measurement should be in x direction
  specific_force(0) = 1.0;
  imu.process_measurement(specific_force, vehicle_angular_velocity);
  imu.get_measurement(gyroscope, accelerometer);
  EXPECT_GT(accelerometer(0), 0.0);
  EXPECT_EQ(accelerometer(1), 0.0);
  EXPECT_EQ(accelerometer(2), 0.0);

  // If force is in -x direction, then the measurement should be in -x direction
  specific_force(0) = -1.0;
  imu.process_measurement(specific_force, vehicle_angular_velocity);
  imu.get_measurement(gyroscope, accelerometer);
  EXPECT_LT(accelerometer(0), 0.0);
  EXPECT_EQ(accelerometer(1), 0.0);
  EXPECT_EQ(accelerometer(2), 0.0);

  // If force is in y direction, then the measurement should be in y direction
  specific_force(0) = 0.0;
  specific_force(1) = 1.0;
  imu.process_measurement(specific_force, vehicle_angular_velocity);
  imu.get_measurement(gyroscope, accelerometer);
  EXPECT_EQ(accelerometer(0), 0.0);
  EXPECT_GT(accelerometer(1), 0.0);
  EXPECT_EQ(accelerometer(2), 0.0);

  // If force is in -y direction, then the measurement should be in -y direction
  specific_force(0) = 0.0;
  specific_force(1) = -1.0;
  imu.process_measurement(specific_force, vehicle_angular_velocity);
  imu.get_measurement(gyroscope, accelerometer);
  EXPECT_EQ(accelerometer(0), 0.0);
  EXPECT_LT(accelerometer(1), 0.0);
  EXPECT_EQ(accelerometer(2), 0.0);

  // If force is in z direction, then the measurement should be in z direction
  specific_force(0) = 0.0;
  specific_force(1) = 0.0;
  specific_force(2) = 1.0;
  imu.process_measurement(specific_force, vehicle_angular_velocity);
  imu.get_measurement(gyroscope, accelerometer);
  EXPECT_EQ(accelerometer(0), 0.0);
  EXPECT_EQ(accelerometer(1), 0.0);
  EXPECT_GT(accelerometer(2), 0.0);

  // If force is in -z direction, then the measurement should be in -z direction
  specific_force(0) = 0.0;
  specific_force(1) = 0.0;
  specific_force(2) = -1.0;
  imu.process_measurement(specific_force, vehicle_angular_velocity);
  imu.get_measurement(gyroscope, accelerometer);
  EXPECT_EQ(accelerometer(0), 0.0);
  EXPECT_EQ(accelerometer(1), 0.0);
  EXPECT_LT(accelerometer(2), 0.0);

  // Test gyroscope measurement
  specific_force           = Eigen::Vector3d::Zero();
  vehicle_angular_velocity = Eigen::Vector3d::Zero();
  gyroscope                = Eigen::Vector3d::Zero();

  // If gyroscope is zero, then the measurement should be zero
  imu.process_measurement(specific_force, vehicle_angular_velocity);
  imu.get_measurement(gyroscope, accelerometer);
  EXPECT_EQ(gyroscope(0), 0.0);
  EXPECT_EQ(gyroscope(1), 0.0);
  EXPECT_EQ(gyroscope(2), 0.0);

  // Angular velocity and measurement sould have the same direction

  // If angular velocity is in x direction, then the measurement should be in x direction
  vehicle_angular_velocity(0) = 1.0;
  imu.process_measurement(specific_force, vehicle_angular_velocity);
  imu.get_measurement(gyroscope, accelerometer);
  EXPECT_GT(gyroscope(0), 0.0);
  EXPECT_EQ(gyroscope(1), 0.0);
  EXPECT_EQ(gyroscope(2), 0.0);

  // If angular velocity is in -x direction, then the measurement should be in -x direction
  vehicle_angular_velocity(0) = -1.0;
  imu.process_measurement(specific_force, vehicle_angular_velocity);
  imu.get_measurement(gyroscope, accelerometer);
  EXPECT_LT(gyroscope(0), 0.0);
  EXPECT_EQ(gyroscope(1), 0.0);
  EXPECT_EQ(gyroscope(2), 0.0);

  // If angular velocity is in y direction, then the measurement should be in y direction
  vehicle_angular_velocity(0) = 0.0;
  vehicle_angular_velocity(1) = 1.0;
  imu.process_measurement(specific_force, vehicle_angular_velocity);
  imu.get_measurement(gyroscope, accelerometer);
  EXPECT_EQ(gyroscope(0), 0.0);
  EXPECT_GT(gyroscope(1), 0.0);
  EXPECT_EQ(gyroscope(2), 0.0);

  // If angular velocity is in -y direction, then the measurement should be in -y direction
  vehicle_angular_velocity(0) = 0.0;
  vehicle_angular_velocity(1) = -1.0;
  imu.process_measurement(specific_force, vehicle_angular_velocity);
  imu.get_measurement(gyroscope, accelerometer);
  EXPECT_EQ(gyroscope(0), 0.0);
  EXPECT_LT(gyroscope(1), 0.0);
  EXPECT_EQ(gyroscope(2), 0.0);

  // If angular velocity is in z direction, then the measurement should be in z direction
  vehicle_angular_velocity(0) = 0.0;
  vehicle_angular_velocity(1) = 0.0;
  vehicle_angular_velocity(2) = 1.0;
  imu.process_measurement(specific_force, vehicle_angular_velocity);
  imu.get_measurement(gyroscope, accelerometer);
  EXPECT_EQ(gyroscope(0), 0.0);
  EXPECT_EQ(gyroscope(1), 0.0);
  EXPECT_GT(gyroscope(2), 0.0);

  // If angular velocity is in -z direction, then the measurement should be in -z direction
  vehicle_angular_velocity(0) = 0.0;
  vehicle_angular_velocity(1) = 0.0;
  vehicle_angular_velocity(2) = -1.0;
  imu.process_measurement(specific_force, vehicle_angular_velocity);
  imu.get_measurement(gyroscope, accelerometer);
  EXPECT_EQ(gyroscope(0), 0.0);
  EXPECT_EQ(gyroscope(1), 0.0);
  EXPECT_LT(gyroscope(2), 0.0);
}

TEST(IMUTest, process_measurement_noise) {
  // Remove noise from the measurements
  double gyro_noise_var                 = 0.001;
  double accel_noise_var                = 0.001;
  double gyro_bias_noise_autocorr_time  = 0;  // No bias for independent measurements
  double accel_bias_noise_autocorr_time = 0;  // No bias for independent measurements
  Eigen::Quaterniond imu_orientation    = Eigen::Quaterniond::Identity();
  Eigen::Vector3d gravity               = Eigen::Vector3d(0, 0, -9.81);
  double dt                             = 0.001;

  // Normal distribution with mean 0 and variance 1
  // Most of the noise should be within 3*std = 3
  // It is multiplied by sqrt(nois_var) to get the correct variance
  // The noise should be within 3*sqrt(noise_var) = 3*sqrt(0.001) = 0.1
  double epsilon = 2 * 0.1;

  // int seed = std::chrono::system_clock::now().time_since_epoch().count();
  int seed = 0;

  PublicIMU<double> imu =
      PublicIMU<double>(gyro_noise_var, accel_noise_var, gyro_bias_noise_autocorr_time,
                        accel_bias_noise_autocorr_time, imu_orientation, gravity, seed);

  imu.reset();

  Eigen::Vector3d gyro_bias  = Eigen::Vector3d::Zero();
  Eigen::Vector3d accel_bias = Eigen::Vector3d::Zero();
  imu.get_bias(gyro_bias, accel_bias);

  // imu.process_bias(dt);

  Eigen::Vector3d specific_force           = Eigen::Vector3d::Zero();
  Eigen::Vector3d vehicle_angular_velocity = Eigen::Vector3d::Zero();
  Eigen::Vector3d gyroscope                = Eigen::Vector3d::Zero();
  Eigen::Vector3d accelerometer            = Eigen::Vector3d::Zero();

  EXPECT_NO_THROW(imu.process_measurement(specific_force, vehicle_angular_velocity));

  // Test accelerometer measurement
  specific_force = Eigen::Vector3d::Zero();
  accelerometer  = Eigen::Vector3d::Zero();

  // If force is zero, then the measurement should be zero
  imu.process_measurement(specific_force, vehicle_angular_velocity);
  imu.get_measurement(gyroscope, accelerometer);
  EXPECT_NEAR(accelerometer(0), 0.0, epsilon);
  EXPECT_NEAR(accelerometer(1), 0.0, epsilon);
  EXPECT_NEAR(accelerometer(2), 0.0, epsilon);

  // Force and measurement sould have the same direction
  double force_magnitude = 10.0;

  // If force is in x direction, then the measurement should be in x direction
  specific_force(0) = force_magnitude;
  imu.process_measurement(specific_force, vehicle_angular_velocity);
  imu.get_measurement(gyroscope, accelerometer);
  EXPECT_GT(accelerometer(0), 0.0);
  EXPECT_NEAR(accelerometer(1), 0.0, epsilon);
  EXPECT_NEAR(accelerometer(2), 0.0, epsilon);

  // If force is in -x direction, then the measurement should be in -x direction
  specific_force(0) = -force_magnitude;
  imu.process_measurement(specific_force, vehicle_angular_velocity);
  imu.get_measurement(gyroscope, accelerometer);
  EXPECT_LT(accelerometer(0), 0.0);
  EXPECT_NEAR(accelerometer(1), 0.0, epsilon);
  EXPECT_NEAR(accelerometer(2), 0.0, epsilon);

  // If force is in y direction, then the measurement should be in y direction
  specific_force(0) = 0.0;
  specific_force(1) = force_magnitude;
  imu.process_measurement(specific_force, vehicle_angular_velocity);
  imu.get_measurement(gyroscope, accelerometer);
  EXPECT_NEAR(accelerometer(0), 0.0, epsilon);
  EXPECT_GT(accelerometer(1), 0.0);
  EXPECT_NEAR(accelerometer(2), 0.0, epsilon);

  // If force is in -y direction, then the measurement should be in -y direction
  specific_force(0) = 0.0;
  specific_force(1) = -force_magnitude;
  imu.process_measurement(specific_force, vehicle_angular_velocity);
  imu.get_measurement(gyroscope, accelerometer);
  EXPECT_NEAR(accelerometer(0), 0.0, epsilon);
  EXPECT_LT(accelerometer(1), 0.0);
  EXPECT_NEAR(accelerometer(2), 0.0, epsilon);

  // If force is in z direction, then the measurement should be in z direction
  specific_force(0) = 0.0;
  specific_force(1) = 0.0;
  specific_force(2) = force_magnitude;
  imu.process_measurement(specific_force, vehicle_angular_velocity);
  imu.get_measurement(gyroscope, accelerometer);
  EXPECT_NEAR(accelerometer(0), 0.0, epsilon);
  EXPECT_NEAR(accelerometer(1), 0.0, epsilon);
  EXPECT_GT(accelerometer(2), 0.0);

  // If force is in -z direction, then the measurement should be in -z direction
  specific_force(0) = 0.0;
  specific_force(1) = 0.0;
  specific_force(2) = -force_magnitude;
  imu.process_measurement(specific_force, vehicle_angular_velocity);
  imu.get_measurement(gyroscope, accelerometer);
  EXPECT_NEAR(accelerometer(0), 0.0, epsilon);
  EXPECT_NEAR(accelerometer(1), 0.0, epsilon);
  EXPECT_LT(accelerometer(2), 0.0);

  // Test gyroscope measurement
  specific_force           = Eigen::Vector3d::Zero();
  vehicle_angular_velocity = Eigen::Vector3d::Zero();
  gyroscope                = Eigen::Vector3d::Zero();

  // If gyroscope is zero, then the measurement should be zero
  imu.process_measurement(specific_force, vehicle_angular_velocity);
  imu.get_measurement(gyroscope, accelerometer);
  EXPECT_NEAR(gyroscope(0), 0.0, epsilon);
  EXPECT_NEAR(gyroscope(1), 0.0, epsilon);
  EXPECT_NEAR(gyroscope(2), 0.0, epsilon);

  // Angular velocity and measurement sould have the same direction
  double angular_velocity_magnitude = 3.0;

  // If angular velocity is in x direction, then the measurement should be in x direction
  vehicle_angular_velocity(0) = angular_velocity_magnitude;
  imu.process_measurement(specific_force, vehicle_angular_velocity);
  imu.get_measurement(gyroscope, accelerometer);
  EXPECT_GT(gyroscope(0), 0.0);
  EXPECT_NEAR(gyroscope(1), 0.0, epsilon);
  EXPECT_NEAR(gyroscope(2), 0.0, epsilon);

  // If angular velocity is in -x direction, then the measurement should be in -x direction
  vehicle_angular_velocity(0) = -angular_velocity_magnitude;
  imu.process_measurement(specific_force, vehicle_angular_velocity);
  imu.get_measurement(gyroscope, accelerometer);
  EXPECT_LT(gyroscope(0), 0.0);
  EXPECT_NEAR(gyroscope(1), 0.0, epsilon);
  EXPECT_NEAR(gyroscope(2), 0.0, epsilon);

  // If angular velocity is in y direction, then the measurement should be in y direction
  vehicle_angular_velocity(0) = 0.0;
  vehicle_angular_velocity(1) = angular_velocity_magnitude;
  imu.process_measurement(specific_force, vehicle_angular_velocity);
  imu.get_measurement(gyroscope, accelerometer);
  EXPECT_NEAR(gyroscope(0), 0.0, epsilon);
  EXPECT_GT(gyroscope(1), 0.0);
  EXPECT_NEAR(gyroscope(2), 0.0, epsilon);

  // If angular velocity is in -y direction, then the measurement should be in -y direction
  vehicle_angular_velocity(0) = 0.0;
  vehicle_angular_velocity(1) = -angular_velocity_magnitude;
  imu.process_measurement(specific_force, vehicle_angular_velocity);
  imu.get_measurement(gyroscope, accelerometer);
  EXPECT_NEAR(gyroscope(0), 0.0, epsilon);
  EXPECT_LT(gyroscope(1), 0.0);
  EXPECT_NEAR(gyroscope(2), 0.0, epsilon);

  // If angular velocity is in z direction, then the measurement should be in z direction
  vehicle_angular_velocity(0) = 0.0;
  vehicle_angular_velocity(1) = 0.0;
  vehicle_angular_velocity(2) = angular_velocity_magnitude;
  imu.process_measurement(specific_force, vehicle_angular_velocity);
  imu.get_measurement(gyroscope, accelerometer);
  EXPECT_NEAR(gyroscope(0), 0.0, epsilon);
  EXPECT_NEAR(gyroscope(1), 0.0, epsilon);
  EXPECT_GT(gyroscope(2), 0.0);

  // If angular velocity is in -z direction, then the measurement should be in -z direction
  vehicle_angular_velocity(0) = 0.0;
  vehicle_angular_velocity(1) = 0.0;
  vehicle_angular_velocity(2) = -angular_velocity_magnitude;
  imu.process_measurement(specific_force, vehicle_angular_velocity);
  imu.get_measurement(gyroscope, accelerometer);
  EXPECT_NEAR(gyroscope(0), 0.0, epsilon);
  EXPECT_NEAR(gyroscope(1), 0.0, epsilon);
  EXPECT_LT(gyroscope(2), 0.0);
}

int main(int argc, char *argv[]) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
