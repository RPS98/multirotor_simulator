/*!*******************************************************************************************
 *  \file       dynamics.hpp
 *  \brief      Dynamics class definition
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

#ifndef DYNAMICS_HPP
#define DYNAMICS_HPP

#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace quadrotor_model {

class Dynamics {
public:
  Dynamics();
  ~Dynamics();

private:
  float mass_;

  float kf_; // Thrust coefficient

  Eigen::Matrix3d inertia_;
  Eigen::Matrix<float, 3, 4> motors_translation_; // From motor to center of mass

  // Motors
  float motor_max_w_;
  float motor_min_w_;


public:
  Eigen::Vector4d motor_w_to_thrust(const Eigen::Vector4d& motor_w) const {
    Eigen::Vector4d thrust;
    thrust << kf_ * motor_w(0) * motor_w(0), kf_ * motor_w(1) * motor_w(1),
        kf_ * motor_w(2) * motor_w(2), kf_ * motor_w(3) * motor_w(3);
    return thrust;
  }

  Eigen::Vector4d motor_thrust_to_w(const Eigen::Vector4d& motor_thrust) const {
    Eigen::Vector4d w;
    w << sqrt(motor_thrust(0) / kf_), sqrt(motor_thrust(1) / kf_),
        sqrt(motor_thrust(2) / kf_), sqrt(motor_thrust(3) / kf_);
    return w;
  }

};  // class Dynamics

}  // namespace quadrotor_model

#endif  // DYNAMICS_HPP