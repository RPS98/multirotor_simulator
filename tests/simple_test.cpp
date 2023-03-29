#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <iostream>
#include <memory>
#include "dynamics.hpp"
#include "quadrotor_actuator.hpp"
#include "quadrotor_state.hpp"
#include "quadrotor_model.hpp"

using namespace quadrotor;

void acro_to_motor_w() {
  actuation::Acro actuator;
  actuator.thrust           = 1.0;
  actuator.angular_velocity = Eigen::Vector3d(1.0, 0.0, 0.0);

  State state_;
  Model model_;

  Eigen::Matrix3d torque_p_gain = Eigen::Vector3d(16.6, 16.6, 5.0).asDiagonal();

  Eigen::Matrix4d b_allocation = Eigen::Matrix4d::Zero();
  b_allocation(0, 0)           = 1.0;
  b_allocation(0, 1)           = 1.0;
  b_allocation(0, 2)           = 1.0;
  b_allocation(0, 3)           = 1.0;
  b_allocation(1, 0)           = -0.1;
  b_allocation(1, 1)           = 0.1;
  b_allocation(1, 2)           = -0.1;
  b_allocation(1, 3)           = 0.1;
  b_allocation(2, 0)           = -0.075;
  b_allocation(2, 1)           = 0.075;
  b_allocation(2, 2)           = 0.075;
  b_allocation(2, 3)           = -0.075;
  b_allocation(3, 0)           = -0.016;
  b_allocation(3, 1)           = -0.016;
  b_allocation(3, 2)           = 0.016;
  b_allocation(3, 3)           = 0.016;

  model_.inertia = Eigen::Vector3d(0.0025, 0.0021, 0.0043).asDiagonal();

  // Angular velocity of the Aircraft error
  const Eigen::Vector3d omega_err = actuator.angular_velocity - state_.kinematics.angular_velocity;

  // Kinetic momentum of a rigid body
  // L = I * w

  // Equation of motion
  // d/dt L = d/dt [I(t) * w(t)]

  // The inertia matrix depends on time, so we can't use the constant inertia matrix
  // d/dt L != I * d/dt w
  // The inertia matrix is a function of the angular velocity
  // d/dt L = I * d/dt w + w x (I * w)
  // where w x (I * w) is the Coriolis effect

  // The d/dt w term is the angular acceleration. It is the control variable with a P controller
  const Eigen::Vector3d angular_acceleration_desired = torque_p_gain * omega_err;

  // The motion equation is:
  // d/dt L = I * angular_acceleration + w x (I * w)
  // The angular acceleration is the control variable, so we can solve for the desired torque
  const Eigen::Vector3d torque_desired =
      model_.inertia * angular_acceleration_desired +
      state_.kinematics.angular_velocity.cross(model_.inertia * state_.kinematics.angular_velocity);

  // Each motor generate a force: F = kf * w²
  // Each motor generate a torque: T = kt * w²
  // The total force is the sum of the forces of each motor
  // F = k * (w1² + w2² + w3² + w4²)
  
  // Each motor generates a torque in the center of the quadrotor: T = F * l = k * w² * l
  // l is the distance between the center of the quadrotor and the motor
  // For a r = radius and theta = angle between the motor and the center of the quadrotor
  // lx = r * sin(theta), ly = r * cos(theta)
  
  // Draw of the quadrotor:
  //          Roll   
  //           ^
  //           |
  //         1 | 4
  // Pitch <---o
  //         2   3 
  // Rotate the quadrotor for math simplification:
  // Roll   
  //  \    /
  //   1  4
  //    \/
  //    /\
  //   2  3
  //  /    \
  // Pitch

  // Roll torque  = L2 * F2 - L4 * F4 = L2 * kf * w2² - L4 * kf * w4² = kf * (L2 * w2² - L4 * w4²)
  // Pitch torque = L1 * F1 - L3 * F3 = L1 * kf * w1² - L3 * kf * w3² = kf * (L1 * w1² - L3 * w3²)
  // Yaw torque   = T1 - T2 + T3 - T4 = kt * w1² - kt * w2² + kt * w3² - kt * w4² = kt * (w1² - w2² + w3² - w4²)

  // [Ft]   [   kf,    kf,    kf,     kf]   [w1²]
  // [tr] = [  0.0, kf*L2,   0.0, -Kf*L4] * [w2²]
  // [tp]   [kf*L1,   0.0, kf*L3,    0.0]   [w3²]
  // [ty]   [   kt,   -kt,    kt,    -kt]   [w4²]

  std::cout << "torque_desired: " << torque_desired.transpose() << std::endl;

  Eigen::Vector4d motor_thrusts =
      b_allocation.inverse() *
      Eigen::Vector4d(actuator.thrust, torque_desired(0), torque_desired(1), torque_desired(2));

  std::cout << "motor_thrusts: " << motor_thrusts.transpose() << std::endl;
}

int main() {
  std::cout << "Hello World!" << std::endl;
  acro_to_motor_w();
  return 0;
}