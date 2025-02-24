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

#ifndef MULTIROTOR_DYNAMIC_MODEL_DYNAMICS_HPP_
#define MULTIROTOR_DYNAMIC_MODEL_DYNAMICS_HPP_

#include <vector>

#include "multirotor_dynamic_model/common/state.hpp"
#include "multirotor_dynamic_model/common/utils.hpp"
#include "multirotor_dynamic_model/model.hpp"

namespace multirotor {

namespace dynamics {

#define LOWEST_UPDATE_FREQUENCY 0.01

/**
 * @brief Struct DynamicsParams
 *
 * Struct containing the parameters of the dynamics
 *
 * @tparam P Precision
 * @tparam num_rotors Number of rotors
 */
template <typename P = double, int num_rotors = 4>
struct DynamicsParams {
  static_assert(std::is_floating_point<P>::value,
                "MotorParams must be used with a floating-point type");
  static_assert(num_rotors > 0, "MotorParams must be used with a positive number of rotors");

  using ModelParams = model::ModelParams<P, num_rotors>;
  using State       = state::State<P, num_rotors>;

  ModelParams model_params = ModelParams();  // Model parameters
  State state              = State();        // Initial state

  int model_seed = 0;  // Seed for the random number generator
};

/**
 * @brief Class Dynamics
 *
 * Class for the dynamics of the multirotor, implementing the derivatives of the
 * state
 *
 * @tparam P Precision
 * @tparam num_rotors Number of rotors
 */
template <typename P = double, int num_rotors = 4>
class Dynamics {
  using Precision   = P;
  using Scalar      = Precision;
  using Vector3     = Eigen::Matrix<Precision, 3, 1>;
  using Vector4     = Eigen::Matrix<Precision, 4, 1>;
  using VectorN     = Eigen::Matrix<Precision, num_rotors, 1>;
  using ModelParams = model::ModelParams<Precision, num_rotors>;
  using Model       = model::Model<Precision, num_rotors>;
  using State       = state::State<Precision, num_rotors>;
  using MotorParams = model::MotorParams<Precision>;

public:
  /**
   * @brief Construct a new Dynamics object
   *
   * @param model Model Model of the aircraft
   * @param state State State of the aircraft
   */
  explicit Dynamics(Model model = Model(), State state = State()) : model_(model), state_(state) {}

  /**
   * @brief Construct a new Dynamics object
   *
   * @param model_params ModelParams Model parameters of the aircraft
   * @param state State State of the aircraft
   */
  Dynamics(ModelParams model_params, State state) : model_(model_params), state_(state) {}

  /**
   * @brief Construct a new Dynamics object
   *
   * @param model_params ModelParams Model parameters of the aircraft
   * @param model_seed Seed for the random number generator
   * @param state State State of the aircraft
   */
  Dynamics(ModelParams model_params, int model_seed, State state)
      : model_(model_params, model_seed), state_(state) {}

  /**
   * @brief Construct a new Dynamics object
   *
   * @param dynamics_params DynamicsParams Dynamics parameters of the aircraft
   */
  explicit Dynamics(DynamicsParams<P, num_rotors> dynamics_params)
      : model_(dynamics_params.model_params, dynamics_params.model_seed),
        state_(dynamics_params.state) {}

  /**
   * @brief Destroy the Dynamics object
   *
   */
  ~Dynamics() {}

  /**
   * @brief Process the dynamics of the system using the Euler explicit method
   *
   * @param actuation_angular_velocity Motor angular velocity (rad/s)
   * @param dt Time step (s)
   * @param external_force External force in earth frame (N)
   * @param enable_noise Enable stochastic noise
   */
  void process_euler_explicit(const VectorN &actuation_angular_velocity,
                              const Scalar dt,
                              const Vector3 &external_force = Vector3::Zero(),
                              const bool enable_noise       = true) {
    if (dt <= 0) {
      throw std::invalid_argument("The time step must be greater than zero");
      return;
    } else if (dt > LOWEST_UPDATE_FREQUENCY) {
      Scalar num_iter = dt / LOWEST_UPDATE_FREQUENCY;
      for (int i = 0; i < num_iter; i++) {
        process_euler_explicit(actuation_angular_velocity, LOWEST_UPDATE_FREQUENCY, external_force,
                               enable_noise);
      }
      return;
    }

    // Initialize stochastic noise
    if (enable_noise) {
      model_.set_stochastic_noise(dt);
    } else {
      model_.disable_stochastic_noise();
    }

    // Process actuation
    VectorN motor_angular_velocity_desired =
        clamp_motors_angular_velocity(actuation_angular_velocity, model_.get_motors());

    // Compute system derivatives
    // Actuators derivatives
    state_.actuators.motor_angular_acceleration = get_motors_angular_velocity_derivative(
        motor_angular_velocity_desired, state_.actuators.motor_angular_velocity,
        model_.get_motors());

    // Kinematics angular derivative
    state_.kinematics.angular_acceleration = get_vehicle_angular_velocity_derivative(
        state_.kinematics.angular_velocity, model_.get_vehicle_inertia(),
        model_.get_torque_by_motors(state_.actuators.motor_angular_velocity,
                                    state_.actuators.motor_angular_acceleration),
        model_.get_aerodynamic_moment(state_.kinematics.angular_velocity),
        model_.get_stochastic_force(), state_.dynamics.torque);

    Vector4 orientation_derivative = get_vehicle_orientation_derivative(
        state_.kinematics.orientation, state_.kinematics.angular_velocity);

    // Kinematics linear derivative
    state_.kinematics.linear_acceleration = get_vehicle_linear_velocity_derivative(
        model_.get_mass(), get_thrust_force_in_earth_frame(state_.actuators.motor_angular_velocity),
        model_.get_drag_force(state_.kinematics.linear_velocity),
        model_.get_mass() * model_.get_gravity(), model_.get_stochastic_force(), external_force,
        state_.dynamics.force);

    Vector3 position_derivative =
        get_vehicle_position_derivative(state_.kinematics.linear_velocity);

    // Integrate system
    // Integrate motors angular velocity
    state_.actuators.motor_angular_velocity += state_.actuators.motor_angular_acceleration * dt;
    state_.actuators.motor_angular_velocity =
        clamp_motors_angular_velocity(state_.actuators.motor_angular_velocity, model_.get_motors());

    // Integrate kinematics linear (earth frame)
    state_.kinematics.position += state_.kinematics.linear_velocity * dt;
    state_.kinematics.linear_velocity += state_.kinematics.linear_acceleration * dt;

    // Integrate kinematics angular (body frame)
    state_.kinematics.orientation.coeffs() += orientation_derivative * dt;
    state_.kinematics.orientation.normalize();
    state_.kinematics.angular_velocity += state_.kinematics.angular_acceleration * dt;
  }

  // Getters

  /**
   * @brief Get the state object
   *
   * @param state State of the aircraft
   */
  inline void get_state(State &state) const { state = state_; }

  /**
   * @brief Get the state object (const version)
   *
   * @return const State& State of the aircraft
   */
  inline const State &get_state() const { return state_; }

  /**
   * @brief Get the model object
   *
   * @param model Model of the aircraft
   */
  inline void get_model(Model &model) const { model = model_; }

  /**
   * @brief Get the model object
   *
   * @return Model Model of the aircraft
   */
  inline Model &get_model() { return model_; }

  /**
   * @brief Get the model object (const version)
   *
   * @return const Model& Model of the aircraft
   */
  const Model &get_model_const() const { return model_; }

  // Setters

  /**
   * @brief Set the state object
   *
   * @param state State of the aircraft
   */
  void set_state(const State &state) { state_ = state; }

  /**
   * @brief Set the state object (pointer version)
   *
   * @param state State of the aircraft
   */
  void set_state(const State *state) {
    if (state != nullptr) {
      state_ = state;
    }
  }

  /**
   * @brief Set the model object
   *
   * @param model Model of the aircraft
   */
  void set_model(const Model &model) { model_ = model; }

  /**
   * @brief Set the model object (pointer version)
   *
   * @param model Model of the aircraft
   */
  void set_model(const Model *model) {
    if (model != nullptr) {
      model_ = model;
    }
  }

protected:
  Model model_;
  State state_;

protected:
  /**
   * @brief Clamp the motors angular velocity
   *
   * Clamp the motors angular velocity between the min and max speed
   *
   * @param motors_angular_velocity Motors angular velocity (rad/s)
   * @param motor_params Vector of motor parameters
   * @return VectorN Motors angular velocity clamped (rad/s)
   */
  static VectorN clamp_motors_angular_velocity(const VectorN &motors_angular_velocity,
                                               const std::vector<MotorParams> &motor_params) {
    VectorN motors_angular_velocity_clamped = motors_angular_velocity;
    for (int i = 0; i < motors_angular_velocity_clamped.size(); i++) {
      motors_angular_velocity_clamped[i] = std::clamp(
          motors_angular_velocity_clamped[i], motor_params[i].min_speed, motor_params[i].max_speed);
    }
    return motors_angular_velocity_clamped;
  }

  /**
   * @brief Get the motors angular velocity derivative as a first order system
   *
   * @param desired_angular_velocity Vector of desired angular velocities
   * (rad/s)
   * @param current_angular_velocity Vector of current angular velocities
   * (rad/s)
   * @param time_constant Time constant of the system (s)
   * @return VectorN Vector of motors angular velocity derivative (rad/s^2)
   */
  static VectorN get_motors_angular_velocity_derivative(
      const VectorN &desired_angular_velocity,
      const VectorN &current_angular_velocity,
      const std::vector<MotorParams> &motor_params) {
    VectorN motor_angular_velocity_derivative = VectorN::Zero(desired_angular_velocity.size());
    for (int i = 0; i < desired_angular_velocity.size(); i++) {
      motor_angular_velocity_derivative[i] =
          (desired_angular_velocity[i] - current_angular_velocity[i]) /
          motor_params[i].time_constant;
    }
    return motor_angular_velocity_derivative;
  }

  /**
   * @brief Compute the vehicle angular velocity derivative in body frame
   *
   * @param vehicle_angular_velocity Vehicle angular velocity in body frame
   * (rad/s)
   * @param vehicle_inertia_matrix Inertia matrix of the vehicle (kg*m^2)
   * @param vehicle_control_moment Vehicle control moment in body frame (N*m)
   * @param vehicle_aerodynamic_moment Vehicle aerodynamic moment in body frame
   * (N*m)
   * @param vehicle_stochastic_force Vehicle stochastic force in body frame
   * (N*m)
   * @param vehicle_total_torque Output vehicle total torque in body frame (N*m)
   *
   * @return Vector3 Vehicle angular velocity derivative in body frame (rad/s^2)
   */
  static Vector3 get_vehicle_angular_velocity_derivative(
      const Vector3 &vehicle_angular_velocity,
      const Eigen::Matrix<P, 3, 3> &vehicle_inertia_matrix,
      const Vector3 &vehicle_control_moment,
      const Vector3 &vehicle_aerodynamic_moment,
      const Vector3 &vehicle_stochastic_force,
      Vector3 &vehicle_total_torque) {
    Vector3 angular_velocity_derivative;

    vehicle_total_torque =
        vehicle_control_moment + vehicle_aerodynamic_moment + vehicle_stochastic_force -
        vehicle_angular_velocity.cross(vehicle_inertia_matrix * vehicle_angular_velocity);

    angular_velocity_derivative = vehicle_inertia_matrix.inverse() * vehicle_total_torque;

    return angular_velocity_derivative;
  }

  /**
   * @brief Get the thrust force in earth frame
   *
   * @param motors_angular_velocity Motors angular velocity in body frame
   * (rad/s)
   * @return Vector3 Thrust force in earth frame (N)
   */
  inline Vector3 get_thrust_force_in_earth_frame(const VectorN &motors_angular_velocity) {
    Vector3 thrust_force_in_body_frame =
        model_.get_force_vector_thrust_by_motors(motors_angular_velocity);

    Vector3 thrust_force_in_earth_frame =
        state_.kinematics.orientation * thrust_force_in_body_frame;
    return thrust_force_in_earth_frame;
  }

  /**
   * @brief Get the vehicle linear velocity derivative
   *
   * @param mass Mass of the vehicle (kg)
   * @param thrust_force Thrust force in earth frame (N)
   * @param drag_force Drag force in earth frame (N)
   * @param gravity_force Gravity force in earth frame (N)
   * @param stochastic_force Stochastic force in earth frame (N)
   * @param external_force External force in earth frame (N)
   * @param vehicle_total_force Output vehicle total force in earth frame (N)
   * @return Vector3 Vehicle linear velocity derivative in earth frame (m/s^2)
   */
  static Vector3 get_vehicle_linear_velocity_derivative(const Scalar mass,
                                                        const Vector3 &thrust_force,
                                                        const Vector3 &drag_force,
                                                        const Vector3 &gravity_force,
                                                        const Vector3 &stochastic_force,
                                                        const Vector3 &external_force,
                                                        Vector3 &vehicle_total_force) {
    // Compute the total force
    vehicle_total_force =
        thrust_force + gravity_force + drag_force + stochastic_force + external_force;

    // Compute the linear velocity derivative
    return vehicle_total_force / mass;
  }

  /**
   * @brief Compute the vehicle orientation derivative
   *
   * @param orientation Vehicle orientation in quaternion (rad)
   * @param angular_velocity Vehicle angular velocity in body frame (rad/s)
   * @return Vector4 Vehicle orientation derivative in quaternion (rad/s)
   */
  static inline Vector4 get_vehicle_orientation_derivative(const Eigen::Quaternion<P> &orientation,
                                                           const Vector3 &angular_velocity) {
    return utils::get_quaternion_derivative(orientation, angular_velocity);
  }

  /**
   * @brief Compute the vehicle position derivative
   *
   * @param linear_velocity Vehicle linear velocity in earth frame (m/s)
   * @return Vector3 Vehicle position derivative in earth frame (m/s^2)
   */
  inline Vector3 get_vehicle_position_derivative(const Vector3 &linear_velocity) {
    return linear_velocity;
  }
};

}  // namespace dynamics

}  // namespace multirotor

#endif  // MULTIROTOR_DYNAMIC_MODEL_DYNAMICS_HPP_
