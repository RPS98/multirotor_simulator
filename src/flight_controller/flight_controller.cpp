/*!*******************************************************************************************
 *  \file       flight_controller.cpp
 *  \brief      Flight Controller implementation file.
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

#include "flight_controller/flight_controller.hpp"

using namespace quadrotor;

FlightController::FlightController(std::shared_ptr<Model> model,
                                   Eigen::Vector3f kp,
                                   Eigen::Vector3f ki = Eigen::Vector3f(0.0, 0.0, 0.0),
                                   Eigen::Vector3f kd = Eigen::Vector3f(0.0, 0.0, 0.0))
    : model_(model) {
  Eigen::Vector3d _kp = kp.cast<double>();
  Eigen::Vector3d _ki = ki.cast<double>();
  Eigen::Vector3d _kd = kd.cast<double>();
  pid_controller_.setGains(_kp, _ki, _kd);
}

FlightController::~FlightController() {
  // TODO
}

void FlightController::acro_to_motor_speeds(const actuation::Acro& acro,
                                            const Eigen::Vector3f& current_angular_velocity,
                                            actuation::MotorW& motor_speeds) {
  // Normalized thrust to Thrust in Newtons
  float thrust = acro.thrust * model_->get_mass();

  // Angular velocity error

  // PID controller for angular velocity error
  Eigen::Vector3f angular_acceleration =
      pid_controller_
          .computeControl(1.0f, current_angular_velocity.cast<double>(),
                          acro.angular_velocity.cast<double>())
          .cast<float>();

  // Use rigid body dynamics to compute the desired motor speeds

  // Compute the desired torque: L = I * dw/dt + w x (I * w)
  Eigen::Vector3f torque_desired =
      model_->get_vehicle_inertia() * angular_acceleration +
      acro.angular_velocity.cross(model_->get_vehicle_inertia() * acro.angular_velocity);

  // Convert from torque to motor speeds using the motor configuration
  // [Ft]   [   kf,    kf,    kf,    kf]   [w1²]
  // [tx] = [  0.0,  l*kt,   0.0, -l*kt] * [w2²]
  // [ty]   [-l*kt,   0.0,  l*kt,   0.0]   [w3²]
  // [tz]   [   kt,   -kt,    kt,   -kt]   [w4²]
  // where l = arm length, kt = torque coefficient, kf = thrust coefficient
  // and Ft = total thrust, tx = torque around x, ty = torque around y, tz = torque around z
  // and w1, w2, w3, w4 = motor speeds
  // Reorder the matrix to get:
  // [w1²]   [   kf,    kf,    kf,    kf]^-1    [Ft]
  // [w2²] = [  0.0,  l*kt,   0.0, -l*kt]     * [tx]
  // [w3²]   [-l*kt,   0.0,  l*kt,   0.0]       [ty]
  // [w4²]   [   kt,   -kt,    kt,   -kt]       [tz]

  // From Model class, we have:

  // [w1²]   [  thrust_c,  thrust_c,  thrust_c, thrust_c]^-1   [Ft]
  // [w2²] = [[ motors_frame_thrust_coefficient_matrix ]]    * [tx]
  // [w3²]   [[                +                       ]]      [ty]
  // [w4²]   [[ motors_frame_torque_coefficient_matrix ]]      [tz]

  Eigen::Matrix4f allocation_matrix   = Eigen::Matrix4f::Zero();
  allocation_matrix.block<1, 4>(0, 0) = Eigen::Vector4f(
      model_->get_motors_thrust_coefficient(), model_->get_motors_thrust_coefficient(),
      model_->get_motors_thrust_coefficient(), model_->get_motors_thrust_coefficient());
  allocation_matrix.block<3, 4>(1, 0) = model_->get_motors_frame_thrust_coefficient_matrix();
  allocation_matrix.block<3, 4>(1, 0) += model_->get_motors_frame_torque_coefficient_matrix();

  // Compute the desired motor speeds
  motor_speeds.angular_velocity =
      allocation_matrix.inverse() *
      Eigen::Vector4f(thrust, torque_desired.x(), torque_desired.y(), torque_desired.z());
}
