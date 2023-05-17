#include "quadrotor_model/common/model.hpp"
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <fstream>
#include <memory>
#include "quadrotor_model/common/state.hpp"
#include "quadrotor_model/quadrotor.hpp"

using namespace quadrotor;

#define LOG(x, y) std::cout << x << ":" << std::endl << y << std::endl

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
      params.motor_thrust_coefficient, params.motor_torque_coefficient, params.motor_dx,
      params.motor_dy, params.motor_min_speed, params.motor_max_speed, params.motor_time_constant,
      params.motor_rotational_inertia, params.motors_frame_type, params.vehicle_mass,
      params.vehicle_inertia, params.vehicle_drag_coefficient,
      params.vehicle_aero_moment_coefficient, params.gravity,
      params.moment_process_noise_auto_correlation, params.force_process_noise_auto_correlation);

  return model;
}

void test_getters() {
  Model model = test_model_instance();

  LOG("get_gravity_force", model.get_gravity_force());
  LOG("get_mass", model.get_mass());
  LOG("get_motors_frame_inertia_matrix", model.get_motors_frame_inertia_matrix());
  LOG("get_motors_frame_thrust_coefficient_matrix",
      model.get_motors_frame_thrust_coefficient_matrix());
  LOG("get_motors_frame_torque_coefficient_matrix",
      model.get_motors_frame_torque_coefficient_matrix());
  LOG("get_motors_thrust_coefficient", model.get_motors_thrust_coefficient());
  LOG("get_motors_frame_type", model.get_motors_frame_type());
  LOG("get_stochastic_force", model.get_stochastic_force());
  LOG("get_stochastic_moment", model.get_stochastic_moment());
  LOG("get_vehicle_inertia", model.get_vehicle_inertia());

  return;
}

void test_methods() {
  Model model = test_model_instance();

  LOG("get_aerodynamic_moment", model.get_aerodynamic_moment(Eigen::Vector3f::Ones()));
  LOG("get_drag_force", model.get_drag_force(Eigen::Vector3f::Ones()));
  LOG("get_motor_torque", model.get_motor_torque(Eigen::Vector4f::Ones(), Eigen::Vector4f::Ones()));
  LOG("get_thrust_force", model.get_thrust_force(Eigen::Vector4f::Ones()));
}

void test_get_motor_torque() {
  Model model = test_model_instance();

  Eigen::Vector4f motors_angular_velocity     = Eigen::Vector4f::Ones();
  Eigen::Vector4f motors_angular_acceleration = Eigen::Vector4f::Zero();

  float low_speed  = 1000.0f;
  float mid_speed  = 2000.0f;
  float high_speed = 3000.0f;

  // Move up
  motors_angular_velocity = Eigen::Vector4f(mid_speed, mid_speed, mid_speed, mid_speed);
  LOG("get_motor_torque, move up",
      model.get_motor_torque(motors_angular_velocity, motors_angular_acceleration));

  // Move forward
  motors_angular_velocity = Eigen::Vector4f(low_speed, mid_speed, high_speed, mid_speed);
  LOG("get_motor_torque, move forward",
      model.get_motor_torque(motors_angular_velocity, motors_angular_acceleration));

  // Move backward
  motors_angular_velocity = Eigen::Vector4f(high_speed, mid_speed, low_speed, mid_speed);
  LOG("get_motor_torque, move backward",
      model.get_motor_torque(motors_angular_velocity, motors_angular_acceleration));

  // Move left
  motors_angular_velocity = Eigen::Vector4f(mid_speed, low_speed, mid_speed, high_speed);
  LOG("get_motor_torque, move left",
      model.get_motor_torque(motors_angular_velocity, motors_angular_acceleration));

  // Move right
  motors_angular_velocity = Eigen::Vector4f(mid_speed, high_speed, mid_speed, low_speed);
  LOG("get_motor_torque, move right",
      model.get_motor_torque(motors_angular_velocity, motors_angular_acceleration));

  // Rotate clockwise
  motors_angular_velocity = Eigen::Vector4f(low_speed, high_speed, low_speed, high_speed);
  LOG("get_motor_torque, rotate clockwise",
      model.get_motor_torque(motors_angular_velocity, motors_angular_acceleration));

  // Rotate counter-clockwise
  motors_angular_velocity = Eigen::Vector4f(high_speed, low_speed, high_speed, low_speed);
  LOG("get_motor_torque, rotate counter-clockwise",
      model.get_motor_torque(motors_angular_velocity, motors_angular_acceleration));

  // Move diagonally forward left
  motors_angular_velocity = Eigen::Vector4f(low_speed, low_speed, high_speed, high_speed);
  LOG("get_motor_torque, move diagonally forward left",
      model.get_motor_torque(motors_angular_velocity, motors_angular_acceleration));

  // Move diagonally forward right
  motors_angular_velocity = Eigen::Vector4f(low_speed, high_speed, high_speed, low_speed);
  LOG("get_motor_torque, move diagonally forward right",
      model.get_motor_torque(motors_angular_velocity, motors_angular_acceleration));

  // Move diagonally backward left
  motors_angular_velocity = Eigen::Vector4f(high_speed, low_speed, low_speed, high_speed);
  LOG("get_motor_torque, move diagonally backward left",
      model.get_motor_torque(motors_angular_velocity, motors_angular_acceleration));

  // Move diagonally backward right
  motors_angular_velocity = Eigen::Vector4f(high_speed, high_speed, low_speed, low_speed);
  LOG("get_motor_torque, move diagonally backward right",
      model.get_motor_torque(motors_angular_velocity, motors_angular_acceleration));
}

int main(int argc, char *argv[]) {
  test_model_instance();
  test_getters();
  test_methods();
  test_get_motor_torque();
  return 0;
}
