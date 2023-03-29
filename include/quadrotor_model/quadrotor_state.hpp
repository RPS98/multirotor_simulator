/*!*******************************************************************************************
 *  \file       quadrotor_state.hpp
 *  \brief      Quadrotor State class definition
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

#ifndef QUADROTOR_STATE_HPP
#define QUADROTOR_STATE_HPP

#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace quadrotor {

namespace state {
struct Kinematics {
  Eigen::Vector3d position             = Eigen::Vector3d::Zero();
  Eigen::Quaterniond orientation       = Eigen::Quaterniond::Identity();
  Eigen::Vector3d linear_velocity      = Eigen::Vector3d::Zero();
  Eigen::Vector3d angular_velocity     = Eigen::Vector3d::Zero();
  Eigen::Vector3d linear_acceleration  = Eigen::Vector3d::Zero();
  Eigen::Vector3d angular_acceleration = Eigen::Vector3d::Zero();
};  // struct Kinematics

struct Dynamics {
  Eigen::Vector3d force  = Eigen::Vector3d::Zero();
  Eigen::Vector3d torque = Eigen::Vector3d::Zero();
};  // struct Dynamics

struct Actuation {
  Eigen::Vector4d motor_w = Eigen::Vector4d::Zero();
};  // struct Actuation

}  // namespace state

struct State {
  state::Kinematics kinematics;
  state::Dynamics dynamics;
  state::Actuation actuation;
}; // struct state

}  // namespace quadrotor

#endif  // QUADROTOR_STATE_HPP