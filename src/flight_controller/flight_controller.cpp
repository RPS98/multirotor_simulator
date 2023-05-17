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

#include "quadrotor_model/flight_controller/flight_controller.hpp"

using namespace quadrotor;

FlightController::FlightController(std::shared_ptr<Model> model,
                                   Eigen::Vector3f kp,
                                   Eigen::Vector3f ki,
                                   Eigen::Vector3f kd,
                                   float antiwindup_cte,
                                   float alpha)
    : model_(model) {
  Eigen::Vector3d _kp = kp.cast<double>();
  Eigen::Vector3d _ki = ki.cast<double>();
  Eigen::Vector3d _kd = kd.cast<double>();
  pid_controller_.setGains(_kp, _ki, _kd);
  pid_controller_.setAntiWindup(antiwindup_cte);
  pid_controller_.setAlpha(alpha);

  allocation_matrix_.block<1, 4>(0, 0) = Eigen::Vector4f(
      model_->get_motors_thrust_coefficient(), model_->get_motors_thrust_coefficient(),
      model_->get_motors_thrust_coefficient(), model_->get_motors_thrust_coefficient());
  allocation_matrix_.block<3, 4>(1, 0) = model_->get_motors_frame_thrust_coefficient_matrix();
  allocation_matrix_.block<3, 4>(1, 0) += model_->get_motors_frame_torque_coefficient_matrix();

  allocation_matrix_inv_ = allocation_matrix_.inverse();
}

FlightController::~FlightController() {
  // TODO
}

void FlightController::reset_controller() { pid_controller_.resetController(); }

Eigen::Vector4f FlightController::acro_to_motor_speeds(
    const actuation::Acro& acro,
    const Eigen::Vector3f& current_angular_velocity,
    const float dt) {
  // PID controller for angular velocity error
  angular_acceleration_ = pid_controller_
                              .computeControl(dt, current_angular_velocity.cast<double>(),
                                              acro.angular_velocity.cast<double>())
                              .cast<float>();

  // Use rigid body dynamics to compute the desired motor speeds

  // Compute the desired torque: L = I * dw/dt + w x (I * w)
  torque_desired_ =
      model_->get_vehicle_inertia() * angular_acceleration_ +
      acro.angular_velocity.cross(model_->get_vehicle_inertia() * acro.angular_velocity);

  // Compute the desired motor speeds
  motor_speeds_angular_velocity_ =
      allocation_matrix_inv_ *
      Eigen::Vector4f(acro.thrust, torque_desired_.x(), torque_desired_.y(), torque_desired_.z());

  // Convert motor speed squared to motor speed
  motor_speeds_angular_velocity_ = utils::sqrt_keep_sign(motor_speeds_angular_velocity_);

  return motor_speeds_angular_velocity_;
}
