#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <fstream>
#include <memory>
#include "gtest/gtest.h"
#include "quadrotor_model/common/actuation.hpp"
#include "quadrotor_model/common/model.hpp"
#include "quadrotor_model/dynamics/dynamics.hpp"
#include "quadrotor_model/quadrotor.hpp"

using namespace quadrotor;

#define LOG(x, y) std::cout << x << ":" << std::endl << y << std::endl

std::shared_ptr<Model> get_model(int motors_frame_type) {
  quadrotor_params params;

  params.motor_thrust_coefficient = 1.91e-6f;
  params.motor_torque_coefficient = 2.6e-7f;
  params.motor_dx                 = 0.08f;
  params.motor_dy                 = 0.08f;
  params.motor_min_speed          = 0.0f;
  params.motor_max_speed          = 2200.0f;
  params.motor_time_constant      = 0.02f;
  params.motor_rotational_inertia = 0.0f;               // 6.62e-6f;
  params.motors_frame_type        = motors_frame_type;  // 0: +-Configuration, 1: X-Configuration
  params.vehicle_mass             = 1.0f;
  params.vehicle_inertia          = Eigen::Vector3f(0.0049f, 0.0049f, 0.0069f).asDiagonal();
  params.vehicle_drag_coefficient = 0.0f;  // 0.1f;
  params.vehicle_aero_moment_coefficient =
      Eigen::Vector3f::Zero()
          .asDiagonal();  // Eigen::Vector3f(0.003f, 0.003f, 0.003f).asDiagonal();
  params.gravity = Eigen::Vector3f::Zero();             // Eigen::Vector3f(0.0f, 0.0f, -9.81f);
  params.moment_process_noise_auto_correlation = 0.0f;  // 1.25e-7f;
  params.force_process_noise_auto_correlation  = 0.0f;  // 0.0005f;

  return std::make_shared<Model>(
      params.motor_thrust_coefficient, params.motor_torque_coefficient, params.motor_dx,
      params.motor_dy, params.motor_min_speed, params.motor_max_speed, params.motor_time_constant,
      params.motor_rotational_inertia, params.motors_frame_type, params.vehicle_mass,
      params.vehicle_inertia, params.vehicle_drag_coefficient,
      params.vehicle_aero_moment_coefficient, params.gravity,
      params.moment_process_noise_auto_correlation, params.force_process_noise_auto_correlation);
}

// Class child of Dynamics to access protected methods and convert to public
class PublicDynamics : public Dynamics {
public:
  PublicDynamics(std::shared_ptr<Model> model_ptr_, std::shared_ptr<State> state_ptr_)
      : Dynamics(model_ptr_, state_ptr_) {}

  using Dynamics::get_motors_angular_velocity_derivative;
  using Dynamics::get_vehicle_angular_velocity_derivative;
  using Dynamics::get_vehicle_control_moment;
  using Dynamics::get_vehicle_linear_velocity_derivative;
  using Dynamics::get_vehicle_orientation_derivative;
  using Dynamics::get_vehicle_position_derivative;
};

PublicDynamics test_dynamics_instance(std::shared_ptr<Model> model_ptr_) {
  auto state_ptr_ = std::make_shared<State>();
  auto dynamics_  = std::make_shared<PublicDynamics>(model_ptr_, state_ptr_);
  return *dynamics_;
}

TEST(Dynamics, GetMotorsAngularVelocityDerivate) {
  auto model       = get_model(0);
  PublicDynamics d = test_dynamics_instance(model);

  Eigen::Vector4f desired_angular_velocity = Eigen::Vector4f::Zero();
  Eigen::Vector4f current_angular_velocity = Eigen::Vector4f::Zero();
  Eigen::Vector4f motors_w_d               = Eigen::Vector4f::Zero();
  float time_constant                      = model->get_motors_time_constant();
  float desired_value                      = 0.0f;
  float current_value                      = 0.0f;

  // Keep speed
  motors_w_d = PublicDynamics::get_motors_angular_velocity_derivative(
      desired_angular_velocity, current_angular_velocity, time_constant);
  EXPECT_EQ(motors_w_d, Eigen::Vector4f::Zero());

  // Increase speed
  desired_value = 1500.0f;
  current_value = 1000.0f;
  desired_angular_velocity =
      Eigen::Vector4f(desired_value, desired_value, desired_value, desired_value);
  current_angular_velocity =
      Eigen::Vector4f(current_value, current_value, current_value, current_value);
  motors_w_d = PublicDynamics::get_motors_angular_velocity_derivative(
      desired_angular_velocity, current_angular_velocity, time_constant);
  EXPECT_GT(motors_w_d(0), 0.0f);
  EXPECT_GT(motors_w_d(1), 0.0f);
  EXPECT_GT(motors_w_d(2), 0.0f);
  EXPECT_GT(motors_w_d(3), 0.0f);
  EXPECT_EQ(motors_w_d, (desired_angular_velocity - current_angular_velocity) / time_constant);

  // Decrease speed
  desired_value = 500.0f;
  current_value = 1000.0f;
  desired_angular_velocity =
      Eigen::Vector4f(desired_value, desired_value, desired_value, desired_value);
  current_angular_velocity =
      Eigen::Vector4f(current_value, current_value, current_value, current_value);
  motors_w_d = PublicDynamics::get_motors_angular_velocity_derivative(
      desired_angular_velocity, current_angular_velocity, time_constant);
  EXPECT_LT(motors_w_d(0), 0.0f);
  EXPECT_LT(motors_w_d(1), 0.0f);
  EXPECT_LT(motors_w_d(2), 0.0f);
  EXPECT_LT(motors_w_d(3), 0.0f);
  EXPECT_EQ(motors_w_d, (desired_angular_velocity - current_angular_velocity) / time_constant);
}

int main(int argc, char *argv[]) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
