/*!*******************************************************************************************
 *  \file       model.hpp
 *  \brief      Model class definition
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

#ifndef MULTIROTOR_DYNAMIC_MODEL_MODEL_HPP_
#define MULTIROTOR_DYNAMIC_MODEL_MODEL_HPP_

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <chrono>
#include <iostream>
#include <memory>
#include <random>
#include <vector>

#include "common/utils.hpp"

namespace multirotor {

namespace model {

/**
 * @brief Struct MotorParams
 *
 * Struct with the motor parameters
 *
 * @tparam P Precision type
 */
template <typename P = double>
struct MotorParams {
  // Check if T is a numeric type (floating-point or integral)
  static_assert(std::is_floating_point<P>::value,
                "MotorParams must be used with a floating-point type");
  using Precision     = P;
  using Scalar        = Precision;
  using IsometryTypeP = Eigen::Transform<Precision, 3, Eigen::Isometry>;

  // Motor characteristics
  int motor_rotation_direction = 1;                // 1 for CW, -1 for CCW
  Scalar thrust_coefficient;                       // N / (rad/s)^2
  Scalar torque_coefficient;                       // N · m / (rad/s)^2
  IsometryTypeP pose = IsometryTypeP::Identity();  // Position 3D (m) and orientation (rad)
  Scalar min_speed;                                // rad/s
  Scalar max_speed;                                // rad/s
  Scalar time_constant;                            // s
  Scalar rotational_inertia;                       // kg · m^2
};

/**
 * @brief Struct ModelParams
 *
 * Struct with the multirotor model parameters
 *
 * @tparam T Precision type
 * @tparam num_rotors Number of rotors
 */
template <typename P = double, int num_rotors = 4>
struct ModelParams {
  // Check if T is a numeric type (floating-point or integral)
  static_assert(std::is_floating_point<P>::value,
                "ModelParams must be used with a floating-point type");
  using Precision    = P;
  using Scalar       = Precision;
  using Vector3      = Eigen::Matrix<Precision, 3, 1>;
  using Matrix3      = Eigen::Matrix<Precision, 3, 3>;
  using MotorParamsP = MotorParams<Precision>;

  // Motor characteristics
  std::vector<MotorParamsP> motors_params =
      std::vector<MotorParamsP>(num_rotors);  // Motor parameters

  // Vehicle characteristics
  Scalar vehicle_mass;      // kg
  Matrix3 vehicle_inertia;  // kg · m^2

  // Aerodynamic characteristics
  Scalar vehicle_drag_coefficient         = 0.0;              // N/(m/s)
  Matrix3 vehicle_aero_moment_coefficient = Matrix3::Zero();  // N · m/(rad/s)^2

  // Environment characteristics
  Vector3 gravity                              = Vector3(0.0, 0.0, -9.81);  // m/s^2
  Scalar force_process_noise_auto_correlation  = 0.0;                       // N^2s
  Scalar moment_process_noise_auto_correlation = 0.0;                       // (N · m)^2s
};

/**
 * @brief Class Model
 *
 * Class with the mutlirotor model and its utilities
 *
 * @tparam T Precision type
 * @tparam num_rotors Number of rotors
 */
template <typename P = double, int num_rotors = 4>
class Model {
  using Precision    = P;
  using Scalar       = Precision;
  using Vector3      = Eigen::Matrix<Precision, 3, 1>;
  using VectorN      = Eigen::Matrix<Precision, num_rotors, 1>;
  using Matrix3      = Eigen::Matrix<Precision, 3, 3>;
  using Matrix3N     = Eigen::Matrix<Precision, 3, num_rotors>;
  using Matrix4N     = Eigen::Matrix<Precision, 4, num_rotors>;
  using Matrix6N     = Eigen::Matrix<Precision, 6, num_rotors>;
  using MatrixN4     = Eigen::Matrix<Precision, num_rotors, 4>;
  using MatrixN6     = Eigen::Matrix<Precision, num_rotors, 6>;
  using MotorParamsP = MotorParams<Precision>;
  using ModelParamsP = ModelParams<Precision, num_rotors>;

public:
  /**
   * @brief Construct a multirotor model
   *
   * Class with the mutlirotor model and its utilities
   *
   * @param ModelParams Multirotor model parameters
   * @param seed Seed for the random number generator
   * @return Model
   */
  explicit Model(ModelParamsP params = ModelParamsP(), int seed = 0) {
    // Random number generator
    random_number_generator_.seed(seed);

    // Motor mixer matrix
    update_params(params);
  }

  /**
   * @brief Destroy the Model object
   */
  ~Model() {}

private:
  // Model parameters
  ModelParamsP params_;

  // Motor mixer matrix
  VectorN mixer_force_vector_;     // Fz = Mixer * w^2
  Matrix3N mixer_force_matrix_;    // [Fx, Fy, Fz] = Mixer * a^2
  Matrix3N mixer_torque_matrix_;   // [Mx, My, Mz] = Mixer * a^2
  Matrix3N mixer_inertia_matrix_;  // [Ix, Iy, Iz] = Mixer * a^2
  MatrixN6 mixer_matrix_;          // [Fx, Fy, Fz, Mx, My, Mz] = Mixer * a^2

  // Noise properties
  std::default_random_engine random_number_generator_;  // Random number generator
  std::normal_distribution<P> standard_normal_distribution_ =
      std::normal_distribution<P>(0.0, 1.0);     // Standard normal distribution
  Vector3 stochastic_force_  = Vector3::Zero();  // N
  Vector3 stochastic_moment_ = Vector3::Zero();  // N · m

public:
  // Methods

  /**
   * @brief Update the multirotor model parameters
   *
   * @param ModelParams Multirotor model parameters
   */
  void update_params(ModelParamsP params) {
    // Check params values
    if (!check_params(params)) {
      std::cerr << "ERROR: Invalid multirotor model parameters" << std::endl;
      return;
    }

    params_ = params;

    // Motor mixer matrix
    Matrix6N mixer_matrix_ = compute_mixer_matrix<6>(params.motors_params);

    // Update mixer matrix
    mixer_force_vector_  = mixer_matrix_.row(2);
    mixer_force_matrix_  = mixer_matrix_.block(0, 0, 3, mixer_matrix_.cols());
    mixer_torque_matrix_ = mixer_matrix_.block(3, 0, 3, mixer_matrix_.cols());

    // Update mixing inertia matrix
    mixer_inertia_matrix_ = compute_mixer_inertia_matrix(params.motors_params);
  }

  /**
   * @brief Set the stochastic noise
   *
   * Add stochastic noise to the multirotor model by a normal distribution
   *
   * @tparam DeltaTimeType Precision type of the time step
   * @param dt Time step (s)
   */
  template <typename DeltaTimeType>
  void set_stochastic_noise(const DeltaTimeType dt) {
    // Convert dt to T type
    Scalar dt_t = static_cast<P>(dt);

    if (dt_t <= 0) {
      disable_stochastic_noise();
      return;
    }

    // Standard normal distribution
    stochastic_force_ = Vector3(sqrt(params_.force_process_noise_auto_correlation / dt) *
                                    standard_normal_distribution_(random_number_generator_),
                                sqrt(params_.force_process_noise_auto_correlation / dt) *
                                    standard_normal_distribution_(random_number_generator_),
                                sqrt(params_.force_process_noise_auto_correlation / dt) *
                                    standard_normal_distribution_(random_number_generator_));

    stochastic_moment_ = Vector3(sqrt(params_.moment_process_noise_auto_correlation / dt) *
                                     standard_normal_distribution_(random_number_generator_),
                                 sqrt(params_.moment_process_noise_auto_correlation / dt) *
                                     standard_normal_distribution_(random_number_generator_),
                                 sqrt(params_.moment_process_noise_auto_correlation / dt) *
                                     standard_normal_distribution_(random_number_generator_));
  }

  /**
   * @brief Set the stochastic noise to zero
   *
   * Set the stochastic noise to zero
   *
   * @return void
   */
  void disable_stochastic_noise() {
    stochastic_force_  = Vector3::Zero();
    stochastic_moment_ = Vector3::Zero();
  }

  /**
   * @brief Get the aerodynamic moment
   *
   * Get aerodynamic moment given the aerodynamic moment coefficient and the
   * vehicle angular velocity using a cross product
   *
   * @param vehicle_angular_velocity Vehicle angular velocity (rad/s)
   * @return Vector3 Aerodynamic moment (N · m)
   */
  inline Vector3 get_aerodynamic_moment(const Vector3 &vehicle_angular_velocity) {
    return get_aerodynamic_moment(get_vehicle_aero_moment_coefficient(), vehicle_angular_velocity);
  }

  /**
   * @brief Get the drag force
   *
   * Get drag force given the drag coefficient and the vehicle linear velocity
   *
   * @param _vehicle_linear_velocity Vehicle linear velocity (m/s)
   * @return Vector3 Drag force (N)
   */
  inline Vector3 get_drag_force(const Vector3 &_vehicle_linear_velocity) {
    return get_drag_force(get_vehicle_drag_coefficient(), _vehicle_linear_velocity);
  }

  /**
   * @brief Get the force thrust by motors
   *
   * Get the force thrust by motors given the mixer force vector and the motors
   * angular velocity
   *
   * @param motors_angular_velocity Motors angular velocity (rad/s)
   * @return T Force thrust by motors (N) in Z axis of body frame
   */
  inline Scalar get_force_thrust_by_motors(const VectorN &motors_angular_velocity) {
    return get_force_thrust_by_motors(get_mixer_force_vector(), motors_angular_velocity);
  }

  /**
   * @brief Get the force vector thrust by motors
   *
   * Get the force vector thrust by motors given the mixer force matrix and the
   * motors angular velocity
   *
   * @param motors_angular_velocity Motors angular velocity (rad/s)
   * @return Vector3 Force vector thrust by motors (N) in body frame
   */
  inline Vector3 get_force_vector_thrust_by_motors(const VectorN &motors_angular_velocity) {
    return get_force_vector_thrust_by_motors(get_mixer_force_matrix(), motors_angular_velocity);
  }

  /**
   * @brief Get the torque by motors
   *
   * Get the torque by motors given the mixer torque matrix, mixer inertia
   * matrix, motors angular velocity and motors angular acceleration
   *
   * @param motors_angular_velocity Motors angular velocity (rad/s)
   * @param motors_angular_acceleration Motors angular acceleration (rad/s^2)
   * @return Vector3 Torque by motors (N · m) in body frame
   */
  inline Vector3 get_torque_by_motors(const VectorN &motors_angular_velocity,
                                      const VectorN &motors_angular_acceleration) const {
    return get_torque_by_motors(get_mixer_torque_matrix(), get_mixer_inertia_matrix(),
                                motors_angular_velocity, motors_angular_acceleration);
  }

public:
  // Getters
  /**
   * @brief Get the mode params
   *
   * Get a copy of the model parameters
   *
   * @return ModelParamsP
   */
  inline ModelParamsP get_params() const { return params_; }

  /**
   * @brief Get the motors
   *
   * Get a copy the motors parameters
   *
   * @return std::vector<MotorParamsP>
   */
  inline std::vector<MotorParamsP> get_motors() const { return params_.motors_params; }

  /**
   * @brief Get the num rotors
   *
   * Get the number of rotors
   *
   * @return int Number of rotors
   */
  inline int get_num_rotors() const { return params_.motors_params.size(); }

  /**
   * @brief Get the mass
   *
   * Get the mass
   *
   * @return Scalar Mass (kg)
   */
  inline Scalar get_mass() const { return params_.vehicle_mass; }

  /**
   * @brief Get the vehicle inertia
   *
   * Get the vehicle inertia
   *
   * @return Matrix3 Vehicle inertia (kg · m^2)
   */
  inline Matrix3 get_vehicle_inertia() const { return params_.vehicle_inertia; }

  /**
   * @brief Get the vehicle drag coefficient
   *
   * Get the vehicle drag coefficient
   *
   * @return Scalar Vehicle drag coefficient
   */
  inline Scalar get_vehicle_drag_coefficient() const { return params_.vehicle_drag_coefficient; }

  /**
   * @brief Get the vehicle aero moment coefficient
   *
   * Get the vehicle aero moment coefficient
   *
   * @return Matrix3 Vehicle aero moment coefficient
   */
  inline Matrix3 get_vehicle_aero_moment_coefficient() const {
    return params_.vehicle_aero_moment_coefficient;
  }

  /**
   * @brief Get the gravity
   *
   * Get the gravity vector
   *
   * @return Vector3 Gravity vector (m/s^2)
   */
  inline Vector3 get_gravity() const { return params_.gravity; }

  /**
   * @brief Get the moment process noise auto correlation
   *
   * Get the moment process noise auto correlation
   *
   * @return Scalar Moment process noise auto correlation
   */
  inline Scalar get_moment_process_noise_auto_correlation() const {
    return params_.moment_process_noise_auto_correlation;
  }

  /**
   * @brief Get the force process noise auto correlation
   *
   * Get the force process noise auto correlation
   *
   * @return Scalar Force process noise auto correlation
   */
  inline Scalar get_force_process_noise_auto_correlation() const {
    return params_.force_process_noise_auto_correlation;
  }

  /**
   * @brief Get the stoch force
   *
   * Get the stochastic force
   *
   * @return Vector3 Stochastic force (N)
   */
  inline Vector3 get_stochastic_force() const { return stochastic_force_; }

  /**
   * @brief Get the stoch moment
   *
   * Get the stochastic moment
   *
   * @return Vector3 Stochastic moment (N · m)
   */
  inline Vector3 get_stochastic_moment() const { return stochastic_moment_; }

  /**
   * @brief Get the mixer torque matrix
   *
   * Get the mixer torque matrix
   *
   * @return Matrix3N Mixer torque matrix
   */
  inline Matrix3N get_mixer_torque_matrix() const { return mixer_torque_matrix_; }

  /**
   * @brief Get the mixer force vector
   *
   * Get the mixer force vector
   *
   * @return VectorN Mixer force vector
   */
  inline VectorN get_mixer_force_vector() const { return mixer_force_vector_; }

  /**
   * @brief Get the mixer force matrix
   *
   * Get the mixer force matrix
   *
   * @return Matrix3N Mixer force matrix
   */
  inline Matrix3N get_mixer_force_matrix() const { return mixer_force_matrix_; }

  /**
   * @brief Get the mixer inertia matrix
   *
   * Get the mixer inertia matrix
   *
   * @return Matrix3N Mixer inertia matrix
   */
  inline Matrix3N get_mixer_inertia_matrix() const { return mixer_inertia_matrix_; }

  /**
   * @brief Get the mixer matrix
   *
   * Get the mixer matrix
   *
   * @return MatrixN6 Mixer matrix
   */
  inline MatrixN6 get_mixer_matrix() const { return mixer_matrix_; }

public:
  // Static functions

  /**
   * @brief Check the model parameters
   *
   * Check the model parameters for validity
   *
   * @param params Model parameters
   * @return true Parameters are valid
   * @return false Parameters are invalid
   */
  bool check_params(ModelParamsP params) {
    if (params.vehicle_mass < 0) {
      std::cout << "ERROR: vehicle mass must be positive" << std::endl;
      return false;
    }
    if (params.vehicle_inertia.determinant() < 0) {
      std::cout << "ERROR: vehicle inertia must be positive definite" << std::endl;
      return false;
    }
    if (params.vehicle_drag_coefficient < 0) {
      std::cout << "ERROR: vehicle drag coefficient must be positive" << std::endl;
      return false;
    }
    if (params.vehicle_aero_moment_coefficient.determinant() < 0) {
      std::cout << "ERROR: vehicle aero moment coefficient must be positive definite" << std::endl;
      return false;
    }
    for (auto &motor : params.motors_params) {
      if (motor.max_speed < motor.min_speed) {
        std::cout << "ERROR: motor max speed must be greater than min speed" << std::endl;
        return false;
      }
      if (motor.time_constant < 0) {
        std::cout << "ERROR: motor time constant must be positive" << std::endl;
        return false;
      }
      if (motor.rotational_inertia < 0) {
        std::cout << "ERROR: motor rotational inertia must be positive" << std::endl;
        return false;
      }
      if (motor.thrust_coefficient < 0) {
        std::cout << "ERROR: motor thrust coefficient must be positive" << std::endl;
        return false;
      }
      if (motor.torque_coefficient < 0) {
        std::cout << "ERROR: motor torque coefficient must be positive" << std::endl;
        return false;
      }
    }
    return true;
  }

  /**
   * @brief Create a quadrotor x config
   *
   * Create a quadrotor x config with 4 motors aligned with the 45 degree axis
   * of the body frame in the x-y plane
   *
   * @param thrust_coefficient Thrust coefficient (N/(rad/s)^2)
   * @param torque_coefficient Torque coefficient (N/(rad/s)^2)
   * @param x_dist X distance from center of mass (m)
   * @param y_dist Y distance from center of mass (m)
   * @param min_speed Minimum motor speed (rad/s)
   * @param max_speed Maximum motor speed (rad/s)
   * @param time_constant Time constant (s)
   * @param rotational_inertia Rotational inertia (kg*m^2)
   * @return std::vector<MotorParamsP> Vector of motor parameters
   */
  static std::vector<MotorParamsP> create_quadrotor_x_config(const Scalar thrust_coefficient,
                                                             const Scalar torque_coefficient,
                                                             const Scalar x_dist,
                                                             const Scalar y_dist,
                                                             const Scalar min_speed,
                                                             const Scalar max_speed,
                                                             const Scalar time_constant,
                                                             const Scalar rotational_inertia) {
    std::vector<MotorParamsP> motors;
    motors.reserve(4);

    MotorParamsP motor;
    motor.thrust_coefficient = thrust_coefficient;
    motor.torque_coefficient = torque_coefficient;
    motor.min_speed          = min_speed;
    motor.max_speed          = max_speed;
    motor.time_constant      = time_constant;
    motor.rotational_inertia = rotational_inertia;

    // Motor 1
    motor.motor_rotation_direction = 1;
    motor.pose.translation()       = Vector3(x_dist, -y_dist, 0);
    motors.push_back(motor);

    // Motor 2
    motor.motor_rotation_direction = -1;
    motor.pose.translation()       = Vector3(x_dist, y_dist, 0);
    motors.push_back(motor);

    // Motor 3
    motor.motor_rotation_direction = 1;
    motor.pose.translation()       = Vector3(-x_dist, y_dist, 0);
    motors.push_back(motor);

    // Motor 4
    motor.motor_rotation_direction = -1;
    motor.pose.translation()       = Vector3(-x_dist, -y_dist, 0);
    motors.push_back(motor);

    return motors;
  }

  /**
   * @brief Create a quadrotor plus config
   *
   * Creates a quadrotor plus configuration with 4 motors aligned with the x and
   * y axis
   *
   * @param thrust_coefficient Thrust coefficient (N/(rad/s)^2)
   * @param torque_coefficient Torque coefficient (N/(rad/s)^2)
   * @param x_dist X distance from center of mass (m)
   * @param y_dist Y distance from center of mass (m)
   * @param min_speed Minimum motor speed (rad/s)
   * @param max_speed Maximum motor speed (rad/s)
   * @param time_constant Time constant (s)
   * @param rotational_inertia Rotational inertia (kg*m^2)
   * @return std::vector<MotorParamsP> Vector of motor parameters
   */
  static std::vector<MotorParamsP> create_quadrotor_plus_config(const Scalar thrust_coefficient,
                                                                const Scalar torque_coefficient,
                                                                const Scalar x_dist,
                                                                const Scalar y_dist,
                                                                const Scalar min_speed,
                                                                const Scalar max_speed,
                                                                const Scalar time_constant,
                                                                const Scalar rotational_inertia) {
    std::vector<MotorParamsP> motors;
    motors.reserve(4);

    MotorParamsP motor;
    motor.thrust_coefficient = thrust_coefficient;
    motor.torque_coefficient = torque_coefficient;
    motor.min_speed          = min_speed;
    motor.max_speed          = max_speed;
    motor.time_constant      = time_constant;
    motor.rotational_inertia = rotational_inertia;

    // Motor 1
    motor.motor_rotation_direction = 1;
    motor.pose.translation()       = Vector3(x_dist, 0, 0);
    motors.push_back(motor);

    // Motor 2
    motor.motor_rotation_direction = -1;
    motor.pose.translation()       = Vector3(0, y_dist, 0);
    motors.push_back(motor);

    // Motor 3
    motor.motor_rotation_direction = 1;
    motor.pose.translation()       = Vector3(-x_dist, 0, 0);
    motors.push_back(motor);

    // Motor 4
    motor.motor_rotation_direction = -1;
    motor.pose.translation()       = Vector3(0, -y_dist, 0);
    motors.push_back(motor);

    return motors;
  }

  /**
   * @brief Create a copter config
   *
   * Creates a hexacopter configuration with the motors placed in a hexagon
   *
   * @param thrust_coefficient Thrust coefficient (N/(rad/s)^2)
   * @param torque_coefficient Torque coefficient (N/(rad/s)^2)
   * @param radius Radius of the hexagon (m)
   * @param min_speed Minimum motor speed (rad/s)
   * @param max_speed Maximum motor speed (rad/s)
   * @param time_constant Time constant (s)
   * @param rotational_inertia Rotational inertia (kg*m^2)
   * @return std::vector<MotorParamsP> Vector of motor parameters
   */
  static std::vector<MotorParamsP> create_hexacopter_config(const Scalar thrust_coefficient,
                                                            const Scalar torque_coefficient,
                                                            const Scalar radius,
                                                            const Scalar min_speed,
                                                            const Scalar max_speed,
                                                            const Scalar time_constant,
                                                            const Scalar rotational_inertia) {
    std::vector<MotorParamsP> motors;
    motors.reserve(6);

    MotorParamsP motor;
    motor.thrust_coefficient = thrust_coefficient;
    motor.torque_coefficient = torque_coefficient;
    motor.min_speed          = min_speed;
    motor.max_speed          = max_speed;
    motor.time_constant      = time_constant;
    motor.rotational_inertia = rotational_inertia;

    const Scalar cos_pi_3 = cos(M_PI / 3);
    const Scalar sin_pi_3 = sin(M_PI / 3);

    // Motor 1
    motor.motor_rotation_direction = 1;
    motor.pose.translation()       = Vector3(radius, 0, 0);
    motors.push_back(motor);

    // Motor 2
    motor.motor_rotation_direction = -1;
    motor.pose.translation()       = Vector3(radius * cos_pi_3, radius * sin_pi_3, 0);
    motors.push_back(motor);

    // Motor 3
    motor.motor_rotation_direction = 1;
    motor.pose.translation()       = Vector3(-radius * cos_pi_3, radius * sin_pi_3, 0);
    motors.push_back(motor);

    // Motor 4
    motor.motor_rotation_direction = -1;
    motor.pose.translation()       = Vector3(-radius, 0, 0);
    motors.push_back(motor);

    // Motor 5
    motor.motor_rotation_direction = 1;
    motor.pose.translation()       = Vector3(-radius * cos_pi_3, -radius * sin_pi_3, 0);
    motors.push_back(motor);

    // Motor 6
    motor.motor_rotation_direction = -1;
    motor.pose.translation()       = Vector3(radius * cos_pi_3, -radius * sin_pi_3, 0);
    motors.push_back(motor);

    return motors;
  }

  /**
   * @brief Compute the mixer matrix
   *
   * Compute the mixer matrix. The mixer matrix is a sizexn matrix, being n the
   * number of motors. If size is 4, it assumes that the motors are placed in a
   * 2D plane, and that the rotation of the motors is around the z axis. If size
   * is 6, it assumes that the motors are placed in a 3D plane, and that the
   * rotation of the motors is around the z axis.
   *
   * @tparam size Size of the mixer matrix (4 or 6)
   * @param motors Vector of motor parameters
   * @return Eigen::Matrix<P, size, num_rotors> Mixer matrix
   */
  template <int size = 4>
  static Eigen::Matrix<P, size, num_rotors> compute_mixer_matrix(
      const std::vector<MotorParamsP> &motors) {
    if constexpr (size == 4) {
      return compute_mixer_matrix_4D(motors);
    } else if constexpr (size == 6) {
      return compute_mixer_matrix_6D(motors);
    }
    static_assert(size == 4 || size == 6, "Size of mixer matrix must be 4 or 6");
  }

  /**
   * @brief Compute the mixer matrix
   *
   * Compute the mixer matrix. The mixer matrix is a 4xn matrix, being n the
   * number of motors. It assumes that the motors are placed in a 2D plane, and
   * that the rotation of the motors is aligned with the z axis of the body
   * frame.
   *
   * @param motors Vector of motor parameters
   * @return Matrix4N Mixer matrix
   */
  static Matrix4N compute_mixer_matrix_4D(const std::vector<MotorParamsP> &motors) {
    for (auto motor : motors) {
      // assert(motor.pose.rotation == Eigen::Matrix<Precision, 3, 3>::Identity());
    }
    const int num_motors = motors.size();

    // Create mixer matrix
    // [Fz]
    // [Tx] = mixer_matrix * [wn²]
    // [Ty]
    // [Tz]

    // Mixer matrix is a 6xn matrix, being n the number of motors
    Matrix4N mixer_matrix_ = Matrix4N::Zero();

    for (int i = 0; i < num_motors; i++) {
      int motor_rotation_direction = motors[i].motor_rotation_direction;  // 1 for CW, -1 for CCW
      assert(motor_rotation_direction == 1 || motor_rotation_direction == -1);

      // Compute the contribution of this motor to the forces and torques in the
      // body frame The contribution is proportional to the motor's thrust and
      // torque coefficients, and it depends on the motor's position relative to
      // the multirotor's body frame. Here, we assume the motors only produce
      // forces along the z-axis (thrust) and torques around the x and y axes on
      // body frame.

      // Total force and torque by motor in body frame

      // Force in body frame = thrust_coefficient
      mixer_matrix_(0, i) = motors[i].thrust_coefficient;

      // Torque in body frame
      mixer_matrix_(1, i) = motors[i].pose.translation().y() * motors[i].thrust_coefficient;
      mixer_matrix_(2, i) =
          Precision(-1) * motors[i].pose.translation().x() * motors[i].thrust_coefficient;
      mixer_matrix_(3, i) =
          static_cast<Precision>(motor_rotation_direction) * motors[i].torque_coefficient;
    }
    return mixer_matrix_;
  }

  /**
   * @brief Compute the mixer matrix for a multirotor
   *
   * Compute the mixer matrix for a multirotor. The mixer matrix relates the
   * angular velocities squared of the motors to the forces and torques
   * generated by the motors in the body frame.
   *
   * @param motors Vector of motor parameters
   * @return Matrix6N Mixer matrix
   */
  static Matrix6N compute_mixer_matrix_6D(const std::vector<MotorParamsP> &motors) {
    int num_motors = motors.size();

    // Create mixer matrix
    // [Fx]
    // [Fy]
    // [Fz] = mixer_matrix * [wn²]
    // [Tx]
    // [Ty]
    // [Tz]

    // Mixer matrix is a 6xn matrix, being n the number of motors
    Matrix6N mixer_matrix_ = Matrix6N::Zero();
    for (int i = 0; i < num_motors; i++) {
      int motor_rotation_direction = motors[i].motor_rotation_direction;  // 1 for CW, -1 for CCW
      assert(motor_rotation_direction == 1 || motor_rotation_direction == -1);

      // Compute the contribution of this motor to the forces and torques in the
      // body frame The contribution is proportional to the motor's thrust and
      // torque coefficients, and it depends on the motor's position and
      // orientation relative to the multirotor's body frame. Here, we assume
      // the motors only produce forces along the z-axis (thrust) and torques
      // around the x and y axes of the motor frame.

      // Force in motor frame
      Vector3 force_motor_frame = Vector3(0, 0, motors[i].thrust_coefficient);

      // Force in body frame
      Vector3 force_body_frame = motors[i].pose.rotation() * force_motor_frame;

      // Torque by motor force in body frame
      Vector3 torque_body_frame = motors[i].pose.translation().cross(force_body_frame);

      // Torque by motor rotation in motor frame
      Vector3 torque_motor_frame_rotation = Vector3(
          0, 0, static_cast<Precision>(motor_rotation_direction) * motors[i].torque_coefficient);

      // Torque by motor rotation in body frame
      Vector3 torque_body_frame_rotation = motors[i].pose.rotation() * torque_motor_frame_rotation;

      // Total torque by motor in body frame
      Vector3 torque_body_frame_total = torque_body_frame + torque_body_frame_rotation;

      // Total force and torque by motor in body frame
      Eigen::Matrix<P, 6, 1> motor_contrib;
      motor_contrib << force_body_frame, torque_body_frame_total;

      // Set the motor's contribution to the corresponding columns of the mixer
      // matrix
      mixer_matrix_.col(i) = motor_contrib;
    }
    return mixer_matrix_;
  }

  /**
   * @brief Compute the inertia matrix of the mixer matrix
   *
   * Compute the inertia matrix of the mixer matrix, which is the matrix that
   * relates the squared angular accelerations of the motors and the torques
   * produced by in the body frame because of the Coriolis effect.
   *
   * @param motors Vector of motor parameters
   * @return Matrix3N Inertia matrix of the mixer matrix
   */
  static Matrix3N compute_mixer_inertia_matrix(const std::vector<MotorParamsP> &motors) {
    int num_motors = motors.size();

    Matrix3N mixing_inertia = Matrix3N::Zero(3, num_motors);

    for (int i = 0; i < num_motors; i++) {
      int motor_rotation_direction = motors[i].motor_rotation_direction;  // 1 for CW, -1 for CCW
      assert(motor_rotation_direction == 1 || motor_rotation_direction == -1);

      // Total torque by motor in motor frame
      Vector3 torque_motor_frame =
          Vector3(0, 0, motor_rotation_direction * motors[i].rotational_inertia);

      // Total torque by motor in body frame
      mixing_inertia.col(i) = motors[i].pose.rotation() * torque_motor_frame;
    }
    return mixing_inertia;
  }

  /**
   * @brief Get the aerodynamic moment in body frame
   *
   * Get the aerodynamic moment in body frame using the vehicle aerodynamic
   * coefficient and the vehicle angular velocity Using a simplified model of
   * aerodynamic moment as a linear function of the vehicle angular velocity
   * Computed as: -vehicle_angular_velocity.norm() *
   * vehicle_aerodynamic_coefficient * vehicle_angular_velocity
   *
   * @param vehicle_aerodynamic_coefficient Vehicle aerodynamic coefficient
   * matrix (kg*m²)
   * @param vehicle_angular_velocity Vehicle angular velocity (rad/s)
   * @return Vector3 Aerodynamic moment in body frame (N*m)
   */
  static Vector3 get_aerodynamic_moment(const Matrix3 &vehicle_aerodynamic_coefficient,
                                        const Vector3 &vehicle_angular_velocity) {
    return -1.0f * vehicle_angular_velocity.norm() * vehicle_aerodynamic_coefficient *
           vehicle_angular_velocity;
  }

  /**
   * @brief Get the drag force in body frame
   *
   * Get the drag force in body frame using the vehicle drag coefficient and the
   * vehicle linear velocity Using a simplified model of drag force as a
   * quadratic function of the vehicle linear velocity Computed as:
   * -vehicle_drag_coefficient * vehicle_linear_velocity²
   *
   * @param vehicle_drag_coefficient Vehicle drag coefficient (kg/m)
   * @param _vehicle_linear_velocity Vehicle linear velocity (m/s)
   * @return Vector3 Drag force in body frame (N)
   */
  static Vector3 get_drag_force(const Scalar vehicle_drag_coefficient,
                                const Vector3 &_vehicle_linear_velocity) {
    return -1.0f * vehicle_drag_coefficient * utils::squared_keep_sign(_vehicle_linear_velocity);
  }

  /**
   * @brief Get the force thrust value by motors in z-axis of body frame
   *
   * Get the force generated by the motors and propellers angular velocity and
   * acceleration using the mixer matrix
   *
   * @param mixer_force_vector Mixer force vector
   * @param motors_angular_velocity Motors angular velocity (rad/s)
   * @return T Force thrust value by motors in z-axis of body frame (N)
   */
  static Scalar get_force_thrust_by_motors(const VectorN &mixer_force_vector,
                                           const VectorN &motors_angular_velocity) {
    return mixer_force_vector.dot(utils::squared_keep_sign(motors_angular_velocity));
  }

  /**
   * @brief Get the force vector thrust by motors in body frame
   *
   * Get the force vector generated by the motors and propellers angular
   * velocity and acceleration using the mixer matrix
   *
   * @param mixer_force_matrix Mixer force matrix
   * @param motors_angular_velocity Motors angular velocity (rad/s)
   * @return Vector3 Force vector thrust by motors in body frame (N)
   */
  static Vector3 get_force_vector_thrust_by_motors(const Matrix3N &mixer_force_matrix,
                                                   const VectorN &motors_angular_velocity) {
    return mixer_force_matrix * utils::squared_keep_sign(motors_angular_velocity);
  }

  /**
   * @brief Get the torque by motors in body frame
   *
   * Get the torque generated by the motors and propellers angular velocity and
   * acceleration using the mixer matrix
   *
   * @param mixer_torque_matrix Mixer torque matrix
   * @param mixer_inertia_matrix Mixer inertia matrix
   * @param motors_angular_velocity Motors angular velocity (rad/s)
   * @param motors_angular_acceleration Motors angular acceleration (rad/s²)
   * @return Vector3 Torque by motors in body frame (N*m)
   */
  static Vector3 get_torque_by_motors(const Matrix3N &mixer_torque_matrix,
                                      const Matrix3N &mixer_inertia_matrix,
                                      const VectorN &motors_angular_velocity,
                                      const VectorN &motors_angular_acceleration) {
    Vector3 motor_torque_angular_velocity =
        mixer_torque_matrix * utils::squared_keep_sign(motors_angular_velocity);

    Vector3 motor_torque_angular_acceleration = mixer_inertia_matrix * motors_angular_acceleration;

    return motor_torque_angular_velocity + motor_torque_angular_acceleration;
  }

private:
  /**
   * @brief Get the mixer matrix
   *
   * Get the mixer matrix of size sizexN where N is the number of motors
   *
   * @tparam size Size of the mixer matrix (4 or 6)
   * @return Eigen::Matrix<P, size, num_rotors> Mixer matrix
   */
  template <int size = 4>
  Eigen::Matrix<P, size, num_rotors> get_mixer_matrix_() const {
    int num_motor = mixer_torque_matrix_.cols();
    if constexpr (size == 4) {
      Matrix4N mixer_matrix_4D      = Matrix4N::Zero(4, num_motor);
      mixer_matrix_4D.row(0)        = mixer_force_vector_;
      mixer_matrix_4D.bottomRows(3) = mixer_torque_matrix_;
      return mixer_matrix_4D;
    } else if constexpr (size == 6) {
      Matrix6N mixer_matrix_6D      = Matrix6N::Zero(6, num_motor);
      mixer_matrix_6D.topRows(3)    = mixer_force_matrix_;
      mixer_matrix_6D.bottomRows(3) = mixer_torque_matrix_;
      return mixer_matrix_6D;
    }
    static_assert(size == 4 || size == 6, "ERROR: Mixer matrix size not supported");
  }
};  // class Model

}  // namespace model

}  // namespace multirotor

#endif  // MULTIROTOR_DYNAMIC_MODEL_MODEL_HPP_
