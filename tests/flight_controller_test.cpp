#include "quadrotor_model/flight_controller/flight_controller.hpp"
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <fstream>
#include <memory>
#include "quadrotor_model/common/actuation.hpp"
#include "quadrotor_model/common/model.hpp"
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

FlightController test_flight_controller_instance(std::shared_ptr<Model> model) {
  Eigen::Vector3f kp = Eigen::Vector3f(9.0f, 9.0f, 9.0f);
  Eigen::Vector3f ki = Eigen::Vector3f(3.0f, 3.0f, 3.0f);
  Eigen::Vector3f kd = Eigen::Vector3f(0.3f, 0.3f, 0.3f);

  FlightController flight_controller = FlightController(model, kp, ki, kd);

  return flight_controller;
}

void test_acro_to_motor_speeds(int motors_frame_type) {
  auto model                         = get_model(motors_frame_type);
  FlightController flight_controller = test_flight_controller_instance(model);

  // Input
  actuation::Acro acro;
  acro.thrust                              = 1.0f;
  Eigen::Vector3f current_angular_velocity = Eigen::Vector3f::Zero();

  // Output
  Eigen::Vector4f motor_speeds;

  float speed = 1.0f;
  float dt    = 0.01f;

  // Move up
  acro.angular_velocity = Eigen::Vector3f(0.0f, 0.0f, 0.0f);
  flight_controller.reset_controller();
  motor_speeds = flight_controller.acro_to_motor_speeds(acro, current_angular_velocity, dt);
  LOG("Move up", motor_speeds);

  // Move clockwise
  acro.angular_velocity = Eigen::Vector3f(0.0f, 0.0f, -speed);
  flight_controller.reset_controller();
  motor_speeds = flight_controller.acro_to_motor_speeds(acro, current_angular_velocity, dt);
  LOG("Move clockwise", motor_speeds);

  // Move counter-clockwise
  acro.angular_velocity = Eigen::Vector3f(0.0f, 0.0f, speed);
  flight_controller.reset_controller();
  motor_speeds = flight_controller.acro_to_motor_speeds(acro, current_angular_velocity, dt);
  LOG("Move counter-clockwise", motor_speeds);

  // Move forward
  acro.angular_velocity = Eigen::Vector3f(0.0f, speed, 0.0f);
  flight_controller.reset_controller();
  motor_speeds = flight_controller.acro_to_motor_speeds(acro, current_angular_velocity, dt);
  LOG("Move forward", motor_speeds);

  // Move backward
  acro.angular_velocity = Eigen::Vector3f(0.0f, -speed, 0.0f);
  flight_controller.reset_controller();
  motor_speeds = flight_controller.acro_to_motor_speeds(acro, current_angular_velocity, dt);
  LOG("Move backward", motor_speeds);

  // Move left
  acro.angular_velocity = Eigen::Vector3f(-speed, 0.0f, 0.0f);
  flight_controller.reset_controller();
  motor_speeds = flight_controller.acro_to_motor_speeds(acro, current_angular_velocity, dt);
  LOG("Move left", motor_speeds);

  // Move right
  acro.angular_velocity = Eigen::Vector3f(speed, 0.0f, 0.0f);
  flight_controller.reset_controller();
  motor_speeds = flight_controller.acro_to_motor_speeds(acro, current_angular_velocity, dt);
  LOG("Move right", motor_speeds);

  // Move diagonally forward left
  acro.angular_velocity = Eigen::Vector3f(-speed, speed, 0.0f);
  flight_controller.reset_controller();
  motor_speeds = flight_controller.acro_to_motor_speeds(acro, current_angular_velocity, dt);
  LOG("Move diagonally forward left", motor_speeds);

  // Move diagonally forward right
  acro.angular_velocity = Eigen::Vector3f(speed, speed, 0.0f);
  flight_controller.reset_controller();
  motor_speeds = flight_controller.acro_to_motor_speeds(acro, current_angular_velocity, dt);
  LOG("Move diagonally forward right", motor_speeds);

  // Move diagonally backward right
  acro.angular_velocity = Eigen::Vector3f(speed, -speed, 0.0f);
  flight_controller.reset_controller();
  motor_speeds = flight_controller.acro_to_motor_speeds(acro, current_angular_velocity, dt);
  LOG("Move diagonally backward right", motor_speeds);

  // Move diagonally backward left
  acro.angular_velocity = Eigen::Vector3f(-speed, -speed, 0.0f);
  flight_controller.reset_controller();
  motor_speeds = flight_controller.acro_to_motor_speeds(acro, current_angular_velocity, dt);
  LOG("Move diagonally backward left", motor_speeds);
}

int main(int argc, char* argv[]) {
  // test_acro_to_motor_speeds(0);
  test_acro_to_motor_speeds(1);
  return 0;
}
