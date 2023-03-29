/*!*******************************************************************************************
 *  \file       flight_controller.cpp
 *  \brief      Flight controller implementation file.
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

#include "quadrotor_model/flight_controller.hpp"

using namespace quadrotor;

FlightController::FlightController() {
  // TODO
}

FlightController::~FlightController() {
  // TODO
}

actuation::MotorW FlightController::acro_to_motor_w(const actuation::Acro &actuator) const {
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

  // The desired motor thrust






  // // The d/dt w term is the angular acceleration. 
  // const Eigen::Vector3d angular_acceleration = 
  //   dynamics_.inertia.inverse() * (actuator.torque - state_.angular_velocity.cross(dynamics_.inertia * state_.angular_velocity));

  // const Eigen::Vector3d torque = 
  //   dynamics_.inertia * omega_err + state_.angular_velocity.cross(dynamics_.inertia * state_.angular_velocity);


  // // Angular momentum = J * angular_velocity
  // // dt Angular momentum = J * Angular_acceleration + angular_velocity x (J * angular_velocity) # Coriolis effect

  // // Desired body torque
  // // Desired Torque = J * K_ang_vel_tau(^-1) * (desired_angular_velocity - current_angular_velocity)
  // //                  + desired_angular_velocity x (J * desired_angular_velocity)
  // const Eigen::Vector3d tau_des = 
  //   dynamics_.inertia * torque_p_gain * omega_err
  //   + state_.angular_velocity.cross(dynamics_.inertia * state_.angular_velocity);


}