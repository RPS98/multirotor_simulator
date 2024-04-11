/*!*******************************************************************************************
 *  \file       utils.hpp
 *  \brief      Utils class definition
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

#ifndef MULTIROTOR_DYNAMIC_MODEL_COMMON_UTILS_HPP_
#define MULTIROTOR_DYNAMIC_MODEL_COMMON_UTILS_HPP_

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <algorithm>

namespace multirotor {

namespace utils {

/**
 * @brief Clamp a vector between a minimum and a maximum value
 *
 * This function clamps each element of the input vector within the specified
 * range [min, max]. It modifies the input vector in-place.
 *
 * @tparam VectorType The type of the input vector (Eigen vector type)
 * @param vector The vector to be clamped
 * @param min The minimum value to which each element of the vector will be
 * clamped
 * @param max The maximum value to which each element of the vector will be
 * clamped
 */
template <typename VectorType>
void clamp_vector(VectorType &vector, const float min, const float max) {
  using Scalar = typename VectorType::Scalar;
  static_assert(std::is_convertible_v<float, Scalar>,
                "VectorType::Scalar must be convertible from float");

  for (int i = 0; i < vector.size(); i++) {
    vector(i) = std::clamp(vector(i), static_cast<Scalar>(min), static_cast<Scalar>(max));
  }
}

/**
 * @brief Compute the quaternion derivative.
 *
 * This function computes the quaternion derivative using Hamiltonian product:
 * q_dot = 0.5 * q * omega_q
 *
 * @tparam QuaternionType The type of the input and output quaternions (Eigen
 * quaternion type)
 * @tparam VectorType The type of the angular velocity vector (Eigen vector
 * type)
 * @param q The input quaternion
 * @param omega The angular velocity vector used for integration
 * @return Eigen::Matrix<typename VectorType::Scalar, 4, 1> The quaternion derivative
 */
template <typename QuaternionType, typename VectorType>
Eigen::Matrix<typename VectorType::Scalar, 4, 1> get_quaternion_derivative(
    const QuaternionType &q,
    const Eigen::MatrixBase<VectorType> &omega) {
  using Scalar = typename VectorType::Scalar;
  static_assert(std::is_convertible_v<Scalar, typename QuaternionType::Scalar>,
                "QuaternionType::Scalar must be convertible to VectorType::Scalar");

  // Convert the angular velocity vector to a quaternion
  Eigen::Quaternion<Scalar> omega_q;
  omega_q.w()   = 0;
  omega_q.vec() = omega.template cast<Scalar>();

  // Convert the input quaternion to a quaternion with the same scalar type as
  // the output quaternion
  Eigen::Quaternion<Scalar> q_q;
  q_q.w()   = q.w();
  q_q.vec() = q.vec().template cast<Scalar>();

  // Calculate the quaternion derivative using Hamiltonian product: q_dot = 0.5
  // * q * omega_q
  return static_cast<Scalar>(0.5) * (q_q * omega_q).coeffs();
}

/**
 * @brief Integrate a quaternion
 *
 * This function integrates a quaternion using Euler integration:
 * q_{k+1} = q_k + q_dot * dt
 *
 * @tparam QuaternionType The type of the input and output quaternions (Eigen
 * quaternion type)
 * @tparam VectorType The type of the angular velocity vector (Eigen vector
 * type)
 * @param q The input quaternion
 * @param omega The angular velocity vector used for integration
 * @param dt The integration time step
 * @return Eigen::Quaternion<typename QuaternionType::Scalar> The integrated
 * quaternion
 */
template <typename QuaternionType, typename VectorType>
Eigen::Quaternion<typename QuaternionType::Scalar> get_quaternion_integrate(const QuaternionType &q,
                                                                            const VectorType &omega,
                                                                            const double dt) {
  using Scalar = typename QuaternionType::Scalar;

  static_assert(std::is_convertible_v<Scalar, typename VectorType::Scalar>,
                "VectorType::Scalar must be convertible to QuaternionType::Scalar");

  static_assert(std::is_convertible_v<double, Scalar>,
                "QuaternionType::Scalar must be convertible to double");

  // Calculate the quaternion derivative using the get_quaternion_derivative
  // function
  Eigen::Matrix<Scalar, 4, 1> q_dot =
      get_quaternion_derivative<QuaternionType, VectorType>(q, omega);

  // Integrate the quaternion using euler integration
  Eigen::Quaternion<Scalar> q_integrated;
  q_integrated.coeffs() = q.coeffs() + q_dot * dt;

  // Normalize the resulting quaternion to ensure it remains unit length
  q_integrated.normalize();

  return q_integrated;
}

/**
 * @brief Compute element-wise square and keep the sign.
 *
 * This function computes the square of each element of the input vector and
 * retains the original sign of each element.
 *
 * @tparam VectorType The type of the input vector (Eigen vector type)
 * @param vector The input vector
 * @return VectorType The resulting vector with squared elements and original
 * sign
 */
template <typename VectorType>
VectorType squared_keep_sign(const VectorType &vector) {
  return vector.array().abs2() * vector.array().sign();
}

/**
 * @brief Compute element-wise square root and keep the sign.
 *
 * This function computes the square root of each element of the input vector
 * and retains the original sign of each element.
 *
 * @tparam VectorType The type of the input vector (Eigen vector type)
 * @param vector The input vector
 * @return VectorType The resulting vector with square root of elements and
 * original sign
 */
template <typename VectorType>
VectorType sqrt_keep_sign(const VectorType &vector) {
  // Stored the sign of each element
  VectorType sign = vector.array().sign();

  // Set each element to its absolute value
  VectorType abs_vector = vector.array().abs();

  // Compute the square root of each element
  VectorType sqrt_vector = abs_vector.array().sqrt();

  // Multiply each element by its sign
  return sqrt_vector.array() * sign.array();
}

}  // namespace utils

}  // namespace multirotor

#endif  // MULTIROTOR_DYNAMIC_MODEL_COMMON_UTILS_HPP_
