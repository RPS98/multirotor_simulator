#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <memory>
#include "quadrotor_model/common/actuation.hpp"
#include "quadrotor_model/common/model.hpp"
#include "quadrotor_model/common/state.hpp"
#include "quadrotor_model/dynamics/dynamics.hpp"
#include "quadrotor_model/flight_controller/flight_controller.hpp"
#include "quadrotor_model/imu/imu.hpp"
#include "quadrotor_model/quadrotor.hpp"

void test_actuation_instance() { quadrotor::actuation::Acro actuation; }

void test_model_instance() {
  Eigen::Matrix3f vehicle_inertia                 = Eigen::Matrix3f::Zero();
  Eigen::Matrix3f vehicle_aero_moment_coefficient = Eigen::Matrix3f::Zero();
  Eigen::Vector3f gravity                         = Eigen::Vector3f::Zero();
  auto model_ptr_                                 = std::make_shared<quadrotor::Model>(
      0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, vehicle_inertia, 0.0f,
      vehicle_aero_moment_coefficient, gravity, 0.0f, 0.0f);
}

void test_state_instance() { quadrotor::State state; }

void test_dynamics_instance() {
  Eigen::Matrix3f vehicle_inertia                 = Eigen::Matrix3f::Zero();
  Eigen::Matrix3f vehicle_aero_moment_coefficient = Eigen::Matrix3f::Zero();
  Eigen::Vector3f gravity                         = Eigen::Vector3f::Zero();
  auto model_ptr_                                 = std::make_shared<quadrotor::Model>(
      0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, vehicle_inertia, 0.0f,
      vehicle_aero_moment_coefficient, gravity, 0.0f, 0.0f);

  auto state_ptr_ = std::make_shared<quadrotor::State>();

  auto dynamics_ = std::make_shared<quadrotor::Dynamics>(model_ptr_, state_ptr_);
}

void test_flight_controller_instance() {
  Eigen::Matrix3f vehicle_inertia                 = Eigen::Matrix3f::Zero();
  Eigen::Matrix3f vehicle_aero_moment_coefficient = Eigen::Matrix3f::Zero();
  Eigen::Vector3f gravity                         = Eigen::Vector3f::Zero();
  auto model_ptr_                                 = std::make_shared<quadrotor::Model>(
      0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, vehicle_inertia, 0.0f,
      vehicle_aero_moment_coefficient, gravity, 0.0f, 0.0f);

  Eigen::Vector3f kp_ = Eigen::Vector3f::Ones();
  Eigen::Vector3f ki_ = Eigen::Vector3f::Zero();
  Eigen::Vector3f kd_ = Eigen::Vector3f::Zero();

  auto flight_controller_ =
      std::make_shared<quadrotor::FlightController>(model_ptr_, kp_, ki_, kd_);
}

void test_imu_instance() {
  float gyro_noise_var_                        = 0.0f;
  float accel_noise_var_                       = 0.0f;
  float gyro_bias_noise_autocorr_time_         = 0.0f;
  float accel_bias_noise_autocorr_time_        = 0.0f;
  Eigen::Quaternionf initial_world_orientation = Eigen::Quaternionf::Identity();
  Eigen::Quaternionf imu_orientation           = Eigen::Quaternionf::Identity();

  auto imu_ = std::make_shared<quadrotor::IMU>(
      gyro_noise_var_, accel_noise_var_, gyro_bias_noise_autocorr_time_,
      accel_bias_noise_autocorr_time_, initial_world_orientation, imu_orientation);
}

void test_quadrotor_instance() {
  quadrotor::quadrotor_params params;
  params.floor_height                      = 0.0f;
  params.initial_state.kinematics.position = Eigen::Vector3f::Zero();
  auto quadrotor_                          = std::make_shared<quadrotor::Quadrotor>(params);
}

int main(int argc, char* argv[]) {
  test_actuation_instance();
  test_model_instance();
  test_state_instance();
  test_dynamics_instance();
  test_quadrotor_instance();
  return 0;
}
