/*!*******************************************************************************************
 *  \file       utils.cpp
 *  \brief      Utils implementation file.
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

#include "common/utils.hpp"

using namespace quadrotor;

/**
 * @brief Clamp a vector between a min and a max
 *
 * @param vector Vector to be clamped
 * @param min Minimum value
 * @param max Maximum value
 */
inline void utils::clamp_vector(Eigen::Vector3f &vector, const float min, const float max) {
  for (int i = 0; i < 3; i++) {
    std::clamp(vector(i), min, max);
  }
  return;
}

/**
 * @brief Clamp a vector between a min and a max
 *
 * @param vector Vector to be clamped
 * @param min Minimum value
 * @param max Maximum value
 */
inline void clamp_vector(Eigen::Vector4f &vector, const float min, const float max) {
  for (int i = 0; i < 4; i++) {
    std::clamp(vector(i), min, max);
  }
  return;
}

/**
 * @brief Get the quaternion derivative object
 *
 * @param q Quaternion to be derived
 * @param omega Angular velocity
 *
 * @return Eigen::Vector4f Quaternion derivative
 */
Eigen::Vector4f utils::get_quaternion_derivative(const Eigen::Quaternionf &q,
                                                 const Eigen::Vector3f &omega) {
  Eigen::Quaternionf omega_q;
  omega_q.w()   = 0;
  omega_q.vec() = omega;
  return 0.5f * (q * omega_q).coeffs();
};

/**
 * @brief Get the quaternion integrate
 *
 * @param q Quaternion to be integrated
 * @param omega Angular velocity
 * @param dt Time step
 *
 * @return Eigen::Quaternionf
 */
Eigen::Quaternionf utils::get_quaternion_integrate(const Eigen::Quaternionf &q,
                                                   const Eigen::Vector3f &omega,
                                                   const float dt) {
  Eigen::Vector4f q_derivate = get_quaternion_derivative(q, omega);
  Eigen::Quaternionf quaternion_integrate;
  quaternion_integrate.coeffs() += q_derivate * dt;
  return quaternion_integrate;
};
