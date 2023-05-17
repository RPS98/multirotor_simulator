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

#include "quadrotor_model/common/utils.hpp"

using namespace quadrotor;

void utils::clamp_vector(Eigen::Vector3f &vector, const float min, const float max) {
  for (int i = 0; i < 3; i++) {
    std::clamp(vector(i), min, max);
  }
  return;
}

void utils::clamp_vector(Eigen::Vector4f &vector, const float min, const float max) {
  for (int i = 0; i < 4; i++) {
    vector(i) = std::clamp(vector(i), min, max);
  }
  return;
}

Eigen::Vector4f utils::get_quaternion_derivative(const Eigen::Quaternionf &q,
                                                 const Eigen::Vector3f &omega) {
  Eigen::Quaternionf omega_q;
  omega_q.w()   = 0;
  omega_q.vec() = omega;
  return 0.5f * (q * omega_q).coeffs();
};

Eigen::Quaternionf utils::get_quaternion_integrate(const Eigen::Quaternionf &q,
                                                   const Eigen::Vector3f &omega,
                                                   const float dt) {
  Eigen::Vector4f q_derivate = get_quaternion_derivative(q, omega);
  Eigen::Quaternionf quaternion_integrate;
  quaternion_integrate.coeffs() += q_derivate * dt;
  return quaternion_integrate;
};

/**
 * @brief Compute Vector squared wise product, keeping the sign
 *
 * @param vector Vector to be squared
 *
 * @return Eigen::Vector3f Squared vector
 */
Eigen::Vector3f utils::squared_keep_sign(const Eigen::Vector3f &vector) {
  return vector.array().abs2() * vector.array().sign();
}

/**
 * @brief Compute Vector squared wise product, keeping the sign
 *
 * @param vector Vector to be squared
 *
 * @return Eigen::Vector3f Squared vector
 */
Eigen::Vector4f utils::squared_keep_sign(const Eigen::Vector4f &vector) {
  return vector.array().abs2() * vector.array().sign();
}

/**
 * @brief Compute Vector square root wise product, keeping the sign
 *
 * @param vector Vector to be squared root
 *
 * @return Eigen::Vector3f Squared root vector
 */
Eigen::Vector3f utils::sqrt_keep_sign(const Eigen::Vector3f &vector) {
  // Stored the sign of each element
  Eigen::Vector3f sign = vector.array().sign();

  // Set each element to its absolute value
  Eigen::Vector3f abs_vector = vector.array().abs();

  // Compute the square root of each element
  Eigen::Vector3f sqrt_vector = abs_vector.array().sqrt();

  // Multiply each element by its sign
  return sqrt_vector.array() * sign.array();
}

/**
 * @brief Compute Vector square root wise product, keeping the sign
 *
 * @param vector Vector to be squared root
 *
 * @return Eigen::Vector4f Squared root vector
 */
Eigen::Vector4f utils::sqrt_keep_sign(const Eigen::Vector4f &vector) {
  // Stored the sign of each element
  Eigen::Vector4f sign = vector.array().sign();

  // Set each element to its absolute value
  Eigen::Vector4f abs_vector = vector.array().abs();

  // Compute the square root of each element
  Eigen::Vector4f sqrt_vector = abs_vector.array().sqrt();

  // Multiply each element by its sign
  return sqrt_vector.array() * sign.array();
}