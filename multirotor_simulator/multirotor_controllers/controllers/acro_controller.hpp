/*!*******************************************************************************************
 *  \file       acro_controller.hpp
 *  \brief      AcroController class definition
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

#ifndef MULTIROTOR_CONTROLLERS_CONTROLLERS_ACRO_CONTROLLER_HPP_
#define MULTIROTOR_CONTROLLERS_CONTROLLERS_ACRO_CONTROLLER_HPP_

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <utility>

namespace acro_controller {

/**
 * @brief Geometric controller parameters
 *
 * @tparam P Precision type of the controller
 */
template <typename P = double>
struct AcroControllerParams {
  // Check if P is a numeric type
  static_assert(std::is_floating_point<P>::value,
                "MotorParams must be used with a floating-point type");

  using Scalar  = P;
  using Vector3 = Eigen::Matrix<P, 3, 1>;
  using Matrix3 = Eigen::Matrix<P, 3, 3>;

  Scalar vehicle_mass = 0.0;                       // kg
  Matrix3 kp_rot      = Matrix3::Zero();           // proportional gains for the rotation
  Vector3 gravity     = Vector3(0.0, 0.0, -9.81);  // m/s^2
};

/**
 * @brief Geometric controller class
 *
 * Convert a desired trajectory into a desired attitude and thrust using differential flatness.
 *
 * @tparam P Precision type of the controller
 */
template <typename P = double>
class AcroController {
  // Check if P is a numeric type
  static_assert(std::is_floating_point<P>::value,
                "MotorParams must be used with a floating-point type");

  using Scalar     = P;
  using Vector3    = Eigen::Matrix<P, 3, 1>;
  using Matrix3    = Eigen::Matrix<P, 3, 3>;
  using Quaternion = Eigen::Quaternion<P>;

public:
  /**
   * @brief Construct a new AcroController object
   *
   * @param vehicle_mass Scalar with the vehicle mass
   * @param kp_rot Matrix3 with the proportional gains for the rotation
   * @param gravity Vector3 with the gravity vector
   */
  AcroController(Scalar vehicle_mass,
                 const Matrix3& kp_rot,
                 const Vector3& gravity = Vector3(0.0, 0.0, -9.81))
      : vehicle_mass_(vehicle_mass), kp_rot_(kp_rot), gravity_(gravity) {}

  /**
   * @brief Construct a new AcroController object
   *
   * @param params AcroControllerParams
   */
  explicit AcroController(const AcroControllerParams<P>& params = AcroControllerParams<P>())
      : vehicle_mass_(params.vehicle_mass), kp_rot_(params.kp_rot), gravity_(params.gravity) {}

  /**
   * @brief Destroy the AcroController object
   *
   */
  ~AcroController() {}

  /**
   * @brief Convert a desired linear acceleration into a desired thrust
   *
   * @param current_orientation Matrix3 with the current orientation
   * @param desired_acceleration Vector3 with the desired acceleration (m/s^2) in earth frame
   *
   * @return Vector3 Desired thrust (N) in earth frame
   */
  Vector3 acceleration_to_thrust(const Vector3& desired_acceleration) {
    const Vector3 thrust_ = (desired_acceleration - gravity_) * vehicle_mass_;
    return thrust_;
  }

  /**
   * @brief Convert a desired yaw angle into a desired angular velocity
   *
   * @param current_orientation Matrix3 with the current orientation
   * @param desired_yaw Scalar with the desired yaw (rad) in earth frame
   * @param desired_thrust Vector3 with the desired thrust (N) in earth frame
   *
   * @return Vector3 Desired angular velocity (rad/s) in body frame
   */
  Vector3 yaw_angle_to_angular_velocity(const Matrix3& current_orientation,
                                        const Scalar desired_yaw,
                                        const Vector3 desired_thrust) {
    // Compute desired attitude
    const Vector3 xc_des = Vector3(cos(desired_yaw), sin(desired_yaw), 0.0);
    Vector3 zb_des       = desired_thrust;
    zb_des.normalize();
    const Vector3 yb_des = zb_des.cross(xc_des).normalized();
    const Vector3 xb_des = yb_des.cross(zb_des).normalized();

    // Compute desired rotation matrix
    Matrix3 R_des;
    R_des << xb_des, yb_des, zb_des;

    // Compute rotation matrix error
    const Matrix3 Mat_R_error =
        R_des.transpose() * current_orientation - current_orientation.transpose() * R_des;

    // Compute rotation vector error
    const Vector3 Vec_R_error = Vector3(Mat_R_error(2, 1), Mat_R_error(0, 2), Mat_R_error(1, 0));

    // Compute the rotation error
    const Vector3 rotation_error = 0.5 * Vec_R_error;

    // Compute the angular velocity error
    vehicle_angular_velocity_ = -kp_rot_ * rotation_error;

    return vehicle_angular_velocity_;
  }

  /**
   * @brief Update vehicle mass
   *
   * @param vehicle_mass Scalar with the vehicle mass (kg)
   */
  inline void update_vehicle_mass(const Scalar vehicle_mass) { vehicle_mass_ = vehicle_mass; }

  /**
   * @brief Update gravity vector
   *
   * @param gravity Vector3 with the gravity vector (m/s^2)
   */
  inline void update_gravity(const Vector3 gravity) { gravity_ = gravity; }

  /**
   * @brief Update kp_rot
   *
   * @param kp_rot Matrix3 with the proportional gains for the rotation
   */
  inline void update_kp_rot(const Matrix3 kp_rot) { kp_rot_ = kp_rot; }

  /**
   * @brief Update controller parameters
   *
   * @param params AcroControllerParams
   */
  inline void update_params(const AcroControllerParams<P>& params) {
    update_vehicle_mass(params.vehicle_mass);
    update_gravity(params.gravity);
    update_kp_rot(params.kp_rot);
  }

  // Getters

  /**
   * @brief Get the desired angular velocity
   *
   * @return Vector3 Desired angular velocity (rad/s)
   */
  inline Vector3 get_desired_angular_velocity() const { return vehicle_angular_velocity_; }

  /**
   * @brief Get the desired angular velocity
   *
   * @return constVector3& Desired angular velocity (rad/s)
   */
  inline const Vector3& get_desired_angular_velocity_const() const {
    return vehicle_angular_velocity_;
  }

  /**
   * @brief Get the desired thrust
   *
   * @return Scalar Desired thrust (N)
   */
  inline Scalar get_desired_thrust() const { return thrust_; }

protected:
  // Control
  Matrix3 kp_rot_ = Matrix3::Zero();  // Proportional gains for the rotation

  // Internal variables
  Scalar vehicle_mass_ = 0;                         // Vehicle mass (kg)
  Vector3 gravity_     = Vector3(0.0, 0.0, -9.81);  // Gravity vector (m/s^2)

  Scalar thrust_                    = 0.0;              // Thrust (N)
  Vector3 vehicle_angular_velocity_ = Vector3::Zero();  // Angular velocity (rad/s)
};
}  // namespace acro_controller

#endif  // MULTIROTOR_CONTROLLERS_CONTROLLERS_ACRO_CONTROLLER_HPP_
