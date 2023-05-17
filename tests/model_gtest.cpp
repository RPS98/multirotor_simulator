#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <fstream>
#include <memory>
#include "gtest/gtest.h"
#include "quadrotor_model/common/model.hpp"
#include "quadrotor_model/common/state.hpp"
#include "quadrotor_model/quadrotor.hpp"

using namespace quadrotor;

Model test_model_instance() {
  quadrotor_params params;

  params.motor_thrust_coefficient = 1.91e-6f;
  params.motor_torque_coefficient = 2.6e-7f;
  params.motor_dx                 = 0.08f;
  params.motor_dy                 = 0.08f;
  params.motor_min_speed          = 0.0f;
  params.motor_max_speed          = 2200.0f;
  params.motor_time_constant      = 0.02f;
  params.motor_rotational_inertia = 0.0f;  // 6.62e-6f;
  params.motors_frame_type        = 0;     // + configuration
  params.vehicle_mass             = 1.0f;
  params.vehicle_inertia          = Eigen::Vector3f(0.0049f, 0.0049f, 0.0069f).asDiagonal();
  params.vehicle_drag_coefficient = 0.0f;  // 0.1f;
  params.vehicle_aero_moment_coefficient =
      Eigen::Vector3f::Zero()
          .asDiagonal();  // Eigen::Vector3f(0.003f, 0.003f, 0.003f).asDiagonal();
  params.gravity = Eigen::Vector3f::Zero();             // Eigen::Vector3f(0.0f, 0.0f, -9.81f);
  params.moment_process_noise_auto_correlation = 0.0f;  // 1.25e-7f;
  params.force_process_noise_auto_correlation  = 0.0f;  // 0.0005f;

  Model model = Model(
    params.motor_thrust_coefficient,
    params.motor_torque_coefficient,
    params.motor_dx,
    params.motor_dy,
    params.motor_min_speed,
    params.motor_max_speed,
    params.motor_time_constant,
    params.motor_rotational_inertia,
    params.motors_frame_type,
    params.vehicle_mass,
    params.vehicle_inertia,
    params.vehicle_drag_coefficient,
    params.vehicle_aero_moment_coefficient,
    params.gravity,
    params.moment_process_noise_auto_correlation,
    params.force_process_noise_auto_correlation);

  return model;
}


TEST(Model, MotorTorque) {
  Model model = test_model_instance();

  Eigen::Vector3f motor_torque                = Eigen::Vector3f::Zero();
  Eigen::Vector4f motors_angular_velocity     = Eigen::Vector4f::Ones();
  Eigen::Vector4f motors_angular_acceleration = Eigen::Vector4f::Zero();

  float low_speed  = 1000.0f;
  float mid_speed  = 2000.0f;
  float high_speed = 3000.0f;

  // Move up
  motors_angular_velocity = Eigen::Vector4f(mid_speed, mid_speed, mid_speed, mid_speed);
  motor_torque = model.get_motor_torque(motors_angular_velocity, motors_angular_acceleration);
  EXPECT_EQ(motor_torque.x(), 0.0f);
  EXPECT_EQ(motor_torque.y(), 0.0f);
  EXPECT_EQ(motor_torque.z(), 0.0f);

  // Move forward
  motors_angular_velocity = Eigen::Vector4f(low_speed, mid_speed, high_speed, mid_speed);
  motor_torque = model.get_motor_torque(motors_angular_velocity, motors_angular_acceleration);
  EXPECT_EQ(motor_torque.x(), 0.0f);
  EXPECT_GT(motor_torque.y(), 0.0f);
  EXPECT_GT(motor_torque.z(), 0.0f);

  // Move backward
  motors_angular_velocity = Eigen::Vector4f(high_speed, mid_speed, low_speed, mid_speed);
  motor_torque = model.get_motor_torque(motors_angular_velocity, motors_angular_acceleration);
  EXPECT_EQ(motor_torque.x(), 0.0f);
  EXPECT_LT(motor_torque.y(), 0.0f);
  EXPECT_GT(motor_torque.z(), 0.0f);

  // Move left
  motors_angular_velocity = Eigen::Vector4f(mid_speed, low_speed, mid_speed, high_speed);
  motor_torque = model.get_motor_torque(motors_angular_velocity, motors_angular_acceleration);
  EXPECT_LT(motor_torque.x(), 0.0f);
  EXPECT_EQ(motor_torque.y(), 0.0f);
  EXPECT_LT(motor_torque.z(), 0.0f);

  // Move right
  motors_angular_velocity = Eigen::Vector4f(mid_speed, high_speed, mid_speed, low_speed);
  motor_torque = model.get_motor_torque(motors_angular_velocity, motors_angular_acceleration);
  EXPECT_GT(motor_torque.x(), 0.0f);
  EXPECT_EQ(motor_torque.y(), 0.0f);
  EXPECT_LT(motor_torque.z(), 0.0f);

  // Rotate clockwise
  motors_angular_velocity = Eigen::Vector4f(low_speed, high_speed, low_speed, high_speed);
  motor_torque = model.get_motor_torque(motors_angular_velocity, motors_angular_acceleration);
  EXPECT_EQ(motor_torque.x(), 0.0f);
  EXPECT_EQ(motor_torque.y(), 0.0f);
  EXPECT_LT(motor_torque.z(), 0.0f);

  // Rotate counter-clockwise
  motors_angular_velocity = Eigen::Vector4f(high_speed, low_speed, high_speed, low_speed);
  motor_torque = model.get_motor_torque(motors_angular_velocity, motors_angular_acceleration);
  EXPECT_EQ(motor_torque.x(), 0.0f);
  EXPECT_EQ(motor_torque.y(), 0.0f);
  EXPECT_GT(motor_torque.z(), 0.0f);

  // Move diagonally forward left
  motors_angular_velocity = Eigen::Vector4f(low_speed, low_speed, high_speed, high_speed);
  motor_torque = model.get_motor_torque(motors_angular_velocity, motors_angular_acceleration);
  EXPECT_LT(motor_torque.x(), 0.0f);
  EXPECT_EQ(-1.0f * motor_torque.x(), motor_torque.y());
  EXPECT_EQ(motor_torque.z(), 0.0f);

  // Move diagonally forward right
  motors_angular_velocity = Eigen::Vector4f(low_speed, high_speed, high_speed, low_speed);
  motor_torque = model.get_motor_torque(motors_angular_velocity, motors_angular_acceleration);
  EXPECT_GT(motor_torque.x(), 0.0f);
  EXPECT_EQ(motor_torque.x(), motor_torque.y());
  EXPECT_EQ(motor_torque.z(), 0.0f);

  // Move diagonally backward left
  motors_angular_velocity = Eigen::Vector4f(high_speed, low_speed, low_speed, high_speed);
  motor_torque = model.get_motor_torque(motors_angular_velocity, motors_angular_acceleration);
  EXPECT_LT(motor_torque.x(), 0.0f);
  EXPECT_EQ(motor_torque.x(), motor_torque.y());
  EXPECT_EQ(motor_torque.z(), 0.0f);

  // Move diagonally backward right
  motors_angular_velocity = Eigen::Vector4f(high_speed, high_speed, low_speed, low_speed);
  motor_torque = model.get_motor_torque(motors_angular_velocity, motors_angular_acceleration);
  EXPECT_GT(motor_torque.x(), 0.0f);
  EXPECT_EQ(motor_torque.x(), -1.0f * motor_torque.y());
  EXPECT_EQ(motor_torque.z(), 0.0f);
}

int main(int argc, char *argv[]) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
