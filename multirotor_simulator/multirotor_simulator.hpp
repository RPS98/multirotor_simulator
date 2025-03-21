/*!*******************************************************************************************
 *  \file       multirotor_simulator.hpp
 *  \brief      Class Simulator definition
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

#ifndef MULTIROTOR_SIMULATOR_MULTIROTOR_SIMULATOR_HPP_
#define MULTIROTOR_SIMULATOR_MULTIROTOR_SIMULATOR_HPP_

#include <limits>
#include <optional>
#include "imu_simulator/inertial_odometry.hpp"
#include "multirotor_controllers/multirotor_controllers.hpp"
#include "multirotor_dynamic_model/dynamics.hpp"
#include "multirotor_dynamic_model/model.hpp"

namespace multirotor {

/**
 * @brief Control modes
 *
 * @details
 * MOTOR_W: Actuation is directly the motors angular velocity (rad/s)
 * HOVER: Hover mode
 * ACRO: Acro mode (thrust, angular velocity)
 * TRAJECTORY: Trajectory tracking (position, velocity, acceleration, yaw)
 * VELOCITY: Velocity control (velocity, yaw)
 * POSITION: Position control (position, yaw)
 */
enum ControlMode { HOVER, MOTOR_W, ACRO, TRAJECTORY, VELOCITY, POSITION };

/**
 * @brief Yaw control modes
 *
 * @details
 * ANGLE: Yaw angle control
 * RATE: Yaw rate control
 */
enum YawControlMode { ANGLE, RATE };

/**
 * @brief Simulator parameters
 *
 * @tparam P Precision type of the controller
 * @tparam num_rotors Number of rotors
 */
template <typename P = double, int num_rotors = 4>
struct SimulatorParams {
  static_assert(std::is_floating_point<P>::value,
                "MotorParams must be used with a floating-point type");
  static_assert(num_rotors > 0, "MotorParams must be used with a positive number of rotors");

  using DynamicsParams         = dynamics::DynamicsParams<P, num_rotors>;
  using ControllerParams       = controller::ControllerParams<P, num_rotors>;
  using IMUParams              = imu::IMUParams<P>;
  using InertialOdometryParams = imu::InertialOdometryParams<P>;

  DynamicsParams dynamics_params     = DynamicsParams();    // Dynamics parameters
  ControllerParams controller_params = ControllerParams();  // Controller parameters
  IMUParams imu_params               = IMUParams();         // IMU parameters
  InertialOdometryParams inertial_odometry_params =
      InertialOdometryParams();  // Inertial odometry parameters
};

/**
 * @brief Class Simulator
 *
 * @tparam P Precision
 * @tparam num_rotors Number of rotors
 */
template <typename P = double, int num_rotors = 4>
class Simulator {
  static_assert(std::is_floating_point<P>::value,
                "MotorParams must be used with a floating-point type");

  using Precision  = P;
  using Scalar     = Precision;
  using Vector3    = Eigen::Matrix<Precision, 3, 1>;
  using VectorN    = Eigen::Matrix<Precision, num_rotors, 1>;
  using Quaternion = Eigen::Quaternion<Precision>;
  using Kinematics = state::internal::Kinematics<Precision>;

  // Dynamics
  using State          = state::State<Precision, num_rotors>;
  using DynamicsParams = dynamics::DynamicsParams<P, num_rotors>;
  using Dynamics       = dynamics::Dynamics<Precision, num_rotors>;

  // Controller
  using ControllerParams = controller::ControllerParams<P, num_rotors>;
  using Controller       = controller::Controller<Precision, num_rotors>;

  // IMU
  using IMUParams              = imu::IMUParams<P>;
  using IMU                    = imu::IMU<Precision>;
  using InertialOdometryParams = imu::InertialOdometryParams<P>;
  using InertialOdometry       = imu::InertialOdometry<Precision>;

public:
  Simulator(const SimulatorParams<Precision, num_rotors>& simulator_params =
                SimulatorParams<Precision, num_rotors>())
      : dynamics_(simulator_params.dynamics_params),
        controller_(simulator_params.controller_params), imu_(simulator_params.imu_params),
        inertial_odometry_(simulator_params.inertial_odometry_params) {}

  ~Simulator() {}

  // Update Simulator

  /**
   * @brief Process the dynamics of the system
   *
   * @param dt Scalar Time step (s)
   */
  void update_dynamics(const Scalar dt) {
    dynamics_.process_euler_explicit(actuation_motors_angular_velocity_, dt, external_force_,
                                     !floor_collision_);

    floor_collision_ = false;
    if (floor_collision_enable_ && dynamics_.get_state().kinematics.position.z() <= floor_height_) {
      floor_collision_ = true;

      // Reset state
      State state = State();

      // Kinematics
      state.kinematics.position.x()        = dynamics_.get_state().kinematics.position.x();
      state.kinematics.position.y()        = dynamics_.get_state().kinematics.position.y();
      state.kinematics.position.z()        = floor_height_;
      state.kinematics.orientation         = dynamics_.get_state().kinematics.orientation;
      state.kinematics.linear_velocity     = Vector3::Zero();
      state.kinematics.angular_velocity    = Vector3::Zero();
      state.kinematics.linear_acceleration = Vector3::Zero();

      state.kinematics.position.z() = std::clamp(state.kinematics.position.z(), floor_height_,
                                                 std::numeric_limits<Scalar>::max());
      state.kinematics.linear_velocity.z() =
          std::clamp(dynamics_.get_state().kinematics.linear_velocity.z(), 0.0,
                     std::numeric_limits<Scalar>::max());
      state.kinematics.linear_acceleration.z() =
          std::clamp(dynamics_.get_state().kinematics.linear_acceleration.z(), 0.0,
                     std::numeric_limits<Scalar>::max());

      // Dynamics
      state.dynamics.force  = Vector3::Zero();
      state.dynamics.torque = Vector3::Zero();

      // Actuators
      state.actuators = dynamics_.get_state().actuators;

      // Set state
      dynamics_.set_state(state);

      // Reset IMU and odometry
      imu_.reset();
      inertial_odometry_.set_initial_position(state.kinematics.position);
      inertial_odometry_.set_initial_orientation(state.kinematics.orientation);
      inertial_odometry_.reset();
    }
  }

  /**
   * @brief Process the controller
   * Controller uses the references set by the set_reference methods.
   * By default, the controller uses the current state from the dynamics (ground truth).
   * User can provide a state to use as input for the controller.
   *
   * @param dt Scalar Time step (s)
   * @param state Optional state to use as input for the controller
   */
  void update_controller(const Scalar dt, const std::optional<Kinematics>& state = std::nullopt) {
    const Kinematics& kinematics = state.value_or(dynamics_.get_state().kinematics);
    if (!armed_) {
      set_actuation(VectorN::Zero());
      return;
    }

    if (yaw_control_mode_ == YawControlMode::RATE) {
      // Update the reference yaw angle with the reference yaw rate
      // Need to be updated every time step, because the current yaw is updated
      update_yaw_reference_with_yaw_rate();
    }

    switch (control_mode_) {
      case MOTOR_W: {
        set_actuation(reference_motors_angular_velocity_);
        break;
      }
      case ACRO: {
        set_actuation(controller_.compute_acro_control(
            kinematics.angular_velocity, reference_thrust_, reference_angular_velocity_, dt));
        break;
      }
      case TRAJECTORY: {
        set_actuation(controller_.compute_trajectory_control(
            kinematics.position, kinematics.linear_velocity,
            kinematics.orientation.toRotationMatrix(), kinematics.angular_velocity,
            reference_position_, reference_velocity_, reference_acceleration_, reference_yaw_, dt));
        break;
      }
      case VELOCITY: {
        set_actuation(controller_.compute_velocity_control(
            kinematics.linear_velocity, kinematics.orientation.toRotationMatrix(),
            kinematics.angular_velocity, reference_velocity_, reference_yaw_, dt));
        break;
      }
      case POSITION: {
        set_actuation(controller_.compute_position_control(
            kinematics.position, kinematics.orientation.toRotationMatrix(),
            kinematics.linear_velocity, kinematics.angular_velocity, reference_position_,
            reference_yaw_, dt));
        break;
      }
      case HOVER: {
        set_actuation(controller_.compute_position_control(
            kinematics.position, kinematics.orientation.toRotationMatrix(),
            kinematics.linear_velocity, kinematics.angular_velocity, reference_position_,
            reference_yaw_, dt));
        break;
      }
      default:
        std::cerr << "Control mode not implemented" << std::endl;
        break;
    }
  }

  /**
   * @brief Process the IMU
   *
   * @param dt Scalar Time step (s)
   */
  inline void update_imu(const Scalar dt) {
    // Convert acceleration from world to body frame
    Vector3 imu_a_world_frame =
        dynamics_.get_state().dynamics.force / dynamics_.get_model_const().get_mass() -
        dynamics_.get_model_const().get_gravity();
    Vector3 imu_a_body_frame =
        dynamics_.get_state().kinematics.orientation.inverse() * imu_a_world_frame;
    imu_.update(dt, imu_a_body_frame, dynamics_.get_state().kinematics.angular_velocity);
  }

  /**
   * @brief Process the inertial odometry
   *
   * @param dt Scalar Time step (s)
   */
  inline void update_inertial_odometry(const Scalar dt) {
    inertial_odometry_.update(imu_.get_measurement_gyro(), imu_.get_measurement_accel(), dt);
  }

  // Control Simulator State

  /**
   * @brief Arm the motors
   *
   */
  void arm() {
    armed_                      = true;
    reference_thrust_           = 9.81;
    reference_angular_velocity_ = Vector3::Zero();
    reference_position_.z()     = floor_height_;
  }

  /**
   * @brief Switch off the motors
   *
   */
  void disarm() {
    actuation_motors_angular_velocity_ = VectorN::Zero();
    armed_                             = false;
  }

  /**
   * @brief Enable floor collision
   *
   * @param floor_height Scalar Floor height (m)
   */
  void enable_floor_collision(const Scalar floor_height = 0.0) {
    floor_collision_enable_ = true;
    floor_height_           = floor_height;
    external_force_         = Vector3::Zero();
  }

  /**
   * @brief Disable floor collision
   *
   */
  inline void disable_floor_collision() {
    floor_collision_enable_ = false;
    external_force_         = Vector3::Zero();
  }

  // Set References

  /**
   * @brief Set the control mode
   *
   * @param control_mode ControlMode Control mode
   */
  inline void set_control_mode(const ControlMode& control_mode,
                               const YawControlMode& yaw_control_mode = YawControlMode::ANGLE) {
    // If the control mode is the same, do nothing
    if (control_mode_ == control_mode && yaw_control_mode_ == yaw_control_mode) {
      return;
    }

    control_mode_       = control_mode;
    yaw_control_mode_   = yaw_control_mode;
    reference_position_ = dynamics_.get_state().kinematics.position;
    Scalar roll, pitch;
    state::internal::quaternion_to_Euler(dynamics_.get_state().kinematics.orientation, roll, pitch,
                                         reference_yaw_);
    reference_velocity_         = Eigen::Vector3d::Zero();
    reference_acceleration_     = Eigen::Vector3d::Zero();
    reference_thrust_           = 0.0;
    reference_angular_velocity_ = Eigen::Vector3d::Zero();
  }

  /**
   * @brief Set the refence motors angular velocity
   *
   * @param reference_motors_angular_velocity VectorN Reference motors angular velocity (rad/s)
   */
  inline void set_refence_motors_angular_velocity(
      const VectorN& reference_motors_angular_velocity) {
    reference_motors_angular_velocity_ = reference_motors_angular_velocity;
  }

  /**
   * @brief Set the reference acro
   * All references are in body frame
   *
   * @param reference_thrust Scalar Reference thrust (N) in body frame
   * @param reference_angular_velocity Vector3 Reference angular velocity (rad/s) in body frame
   */
  inline void set_reference_acro(const Scalar reference_thrust,
                                 const Vector3 reference_angular_velocity) {
    reference_thrust_           = reference_thrust;
    reference_angular_velocity_ = reference_angular_velocity;
  }

  /**
   * @brief Set the reference trajectory.
   * All references are in world frame
   *
   * @param reference_position Vector3 Reference position (m) in world frame
   * @param reference_velocity Vector3 Reference velocity (m/s) in world frame
   * @param reference_acceleration Vector3 Reference acceleration (m/s^2) in world frame
   * @param reference_yaw Scalar Reference yaw (rad) in world frame
   */
  inline void set_reference_trajectory(const Vector3& reference_position,
                                       const Vector3& reference_velocity,
                                       const Vector3& reference_acceleration) {
    reference_position_     = reference_position;
    reference_velocity_     = reference_velocity;
    reference_acceleration_ = reference_acceleration;
  }

  /**
   * @brief Set the reference velocity
   * All references are in world frame
   *
   * @param reference_velocity Vector3 Reference velocity (m/s) in world frame
   * @param reference_yaw Scalar Reference yaw (rad) in world frame
   */
  inline void set_reference_velocity(const Vector3& reference_velocity) {
    reference_velocity_ = reference_velocity;
  }

  /**
   * @brief Set the reference position
   * All references are in world frame
   *
   * @param reference_position Vector3 Reference position (m) in world frame
   * @param reference_yaw Scalar Reference yaw (rad) in world frame
   */
  inline void set_reference_position(const Vector3& reference_position) {
    reference_position_ = reference_position;
  }

  /**
   * @brief Set the reference yaw angle in world frame
   *
   * @param reference_yaw Scalar Reference yaw angle (rad) in world frame
   */
  inline void set_reference_yaw_angle(const Scalar reference_yaw) {
    reference_yaw_ = reference_yaw;
  }

  /**
   * @brief Set the reference yaw rate in world frame
   * The reference yaw is updated every time step
   *
   * @param reference_yaw_rate Scalar Reference yaw rate (rad/s) in world frame
   */
  inline void set_reference_yaw_rate(const Scalar reference_yaw_rate) {
    reference_yaw_rate_ = reference_yaw_rate;
  }

  // Get Aircraft State

  /**
   * @brief Get the state
   *
   * @param state State State
   */
  inline void get_state(State& state) const { state = dynamics_.get_state(); }

  /**
   * @brief Get the state
   *
   * @return const State& State
   */
  inline const State& get_state() const { return dynamics_.get_state(); }

  /**
   * @brief Get the inertial odometry
   *
   * @param odometry_kinematics Kinematics Kinematics
   */
  inline void get_odometry(Kinematics& kinematics) const {
    inertial_odometry_.get_measurement(kinematics.position, kinematics.orientation,
                                       kinematics.linear_velocity, kinematics.angular_velocity,
                                       kinematics.linear_acceleration);
  }

  /**
   * @brief Get the inertial odometry
   *
   * @return Kinematics Kinematics
   */
  inline Kinematics get_odometry() const {
    Kinematics kinematics;
    inertial_odometry_.get_measurement(kinematics.position, kinematics.orientation,
                                       kinematics.linear_velocity, kinematics.angular_velocity,
                                       kinematics.linear_acceleration);
    return kinematics;
  }

  /**
   * @brief Get the inertial measurement unit measurement
   *
   * @param gyro Vector3 Gyro measurement (rad/s)
   * @param accel Vector3 Accel measurement (m/s^2)
   */
  inline void get_imu_measurement(Vector3& gyro, Vector3& accel) const {
    imu_.get_measurement(gyro, accel);
  }

  // Get Simulator State

  /**
   * @brief Get the dynamics
   *
   * @param dynamics Dynamics Dynamics
   */
  inline void get_dynamics(Dynamics& dynamics) const { dynamics = dynamics_; }

  /**
   * @brief Get the dynamics
   *
   * @return Dynamics& Dynamics
   */
  inline Dynamics& get_dynamics() { return dynamics_; }

  /**
   * @brief Get the dynamics
   *
   * @return const Dynamics& Dynamics
   */
  inline const Dynamics& get_dynamics_const() const { return dynamics_; }

  /**
   * @brief Get the controller
   *
   * @param controller Controller Controller
   */
  inline void get_controller(Controller& controller) const { controller = controller_; }

  /**
   * @brief Get the controller
   *
   * @return Controller& Controller
   */
  inline Controller& get_controller() { return controller_; }

  /**
   * @brief Get the controller
   *
   * @return const Controller& Controller
   */
  inline const Controller& get_controller_const() const { return controller_; }

  /**
   * @brief Get the imu
   *
   * @param imu IMU IMU
   */
  inline void get_imu(IMU& imu) const { imu = imu_; }

  /**
   * @brief Get the imu
   *
   * @return IMU& IMU
   */
  inline IMU& get_imu() { return imu_; }

  /**
   * @brief Get the imu
   *
   * @return const IMU& IMU
   */
  inline const IMU& get_imu_const() const { return imu_; }

  /**
   * @brief Get the inertial odometry
   *
   * @param inertial_odometry InertialOdometry Inertial odometry
   */
  inline void get_inertial_odometry(InertialOdometry& inertial_odometry) const {
    inertial_odometry = inertial_odometry_;
  }

  /**
   * @brief Get the inertial odometry
   *
   * @return InertialOdometry& Inertial odometry
   */
  inline InertialOdometry& get_inertial_odometry() { return inertial_odometry_; }

  /**
   * @brief Get the inertial odometry
   *
   * @return const InertialOdometry& Inertial odometry
   */
  inline const InertialOdometry& get_inertial_odometry_const() const { return inertial_odometry_; }

  // Setters

  /**
   * @brief Set the dynamics params
   *
   * @param dynamics DynamicsParams Dynamics parameters
   */
  inline void set_dynamics_params(const DynamicsParams& dynamics_params) {
    dynamics_ = Dynamics(dynamics_params);
  }

  /**
   * @brief Set the controller params
   *
   * @param controller ControllerParams Controller parameters
   */
  inline void set_controller_params(const ControllerParams& controller_params) {
    controller_ = Controller(controller_params);
  }

  /**
   * @brief Set the imu params
   *
   * @param imu IMUParams IMU parameters
   */
  inline void set_imu_params(const IMUParams& imu_params) { imu_ = IMU(imu_params); }

  /**
   * @brief Set the inertial odometry params
   *
   * @param inertial_odometry InertialOdometryParams Inertial odometry parameters
   */
  inline void set_inertial_odometry_params(const InertialOdometryParams& inertial_odometry_params) {
    inertial_odometry_ = InertialOdometry(inertial_odometry_params);
  }

protected:
  /**
   * @brief Set the actuation
   *
   * @param actuation_motors_angular_velocity VectorN Actuation motors angular velocity (rad/s)
   */
  inline void set_actuation(const VectorN& actuation_motors_angular_velocity) {
    actuation_motors_angular_velocity_ = actuation_motors_angular_velocity;
  }

  /**
   * @brief Update the yaw reference with the yaw rate
   *
   */
  void update_yaw_reference_with_yaw_rate() {
    // Convert quaternion to Euler angles
    Scalar roll, pitch, current_yaw;
    state::internal::quaternion_to_Euler(dynamics_.get_state().kinematics.orientation, roll, pitch,
                                         current_yaw);
    reference_yaw_ = controller_.yaw_rate_to_angle(current_yaw, reference_yaw_rate_);
  }

private:
  // Internal classes
  Dynamics dynamics_;
  Controller controller_;
  IMU imu_;
  InertialOdometry inertial_odometry_;

  // Simulator state
  bool armed_                  = false;
  bool floor_collision_enable_ = false;
  bool floor_collision_        = false;
  Scalar floor_height_         = 0.0;

  // Motion References
  // Motors angular velocity
  VectorN reference_motors_angular_velocity_ = VectorN::Zero();

  // Acro
  Scalar reference_thrust_            = 0.0;
  Vector3 reference_angular_velocity_ = Vector3::Zero();

  // Trajectory, Position, Velocity
  Vector3 reference_position_     = Vector3::Zero();
  Vector3 reference_velocity_     = Vector3::Zero();
  Vector3 reference_acceleration_ = Vector3::Zero();
  Scalar reference_yaw_           = 0.0;
  Scalar reference_yaw_rate_      = 0.0;

  // Actuation Commands
  VectorN actuation_motors_angular_velocity_ = VectorN::Zero();
  Vector3 external_force_                    = Vector3::Zero();

  // Control mode
  ControlMode control_mode_        = ControlMode::MOTOR_W;
  YawControlMode yaw_control_mode_ = YawControlMode::ANGLE;

public:
  // Getters

  /**
   * @brief Get the arm status
   *
   * @param armed bool Arm status
   */
  inline void get_armed(bool& armed) const { armed = armed_; }

  /**
   * @brief Get the arm status
   *
   * @return bool Arm status
   */
  inline bool get_armed() const { return armed_; }

  /**
   * @brief Get the floor collision enable status
   *
   * @param floor_collision_enable bool Floor collision enable status
   */
  inline void get_floor_collision_enabled(bool& floor_collision_enable) const {
    floor_collision_enable = floor_collision_enable_;
  }

  /**
   * @brief Get the floor collision enable status
   *
   * @return bool Floor collision enable status
   */
  inline bool get_floor_collision_enabled() const { return floor_collision_enable_; }

  /**
   * @brief Get the floor height
   *
   * @param floor_height Scalar Floor height
   */
  inline void get_floor_height(Scalar& floor_height) const { floor_height = floor_height_; }

  /**
   * @brief Get the floor height
   *
   * @return Scalar Floor height
   */
  inline Scalar get_floor_height() const { return floor_height_; }

  /**
   * @brief Get the reference motors angular velocity
   *
   * @param reference_motors_angular_velocity VectorN Reference motors angular velocity
   */
  inline void get_reference_motors_angular_velocity(
      VectorN& reference_motors_angular_velocity) const {
    reference_motors_angular_velocity = reference_motors_angular_velocity_;
  }

  /**
   * @brief Get the reference motors angular velocity
   *
   * @return const VectorN& Reference motors angular velocity
   */
  inline const VectorN& get_reference_motors_angular_velocity() const {
    return reference_motors_angular_velocity_;
  }

  /**
   * @brief Get the reference thrust
   *
   * @param reference_thrust Scalar Reference thrust
   */
  inline void get_reference_thrust(Scalar& reference_thrust) const {
    reference_thrust = reference_thrust_;
  }

  /**
   * @brief Get the reference thrust
   *
   * @return Scalar Reference thrust
   */
  inline Scalar get_reference_thrust() const { return reference_thrust_; }

  /**
   * @brief Get the reference angular velocity
   *
   * @param reference_angular_velocity Vector3 Reference angular velocity
   */
  inline void get_reference_angular_velocity(Vector3& reference_angular_velocity) const {
    reference_angular_velocity = reference_angular_velocity_;
  }

  /**
   * @brief Get the reference angular velocity
   *
   * @return const Vector3& Reference angular velocity
   */
  inline const Vector3& get_reference_angular_velocity() const {
    return reference_angular_velocity_;
  }

  /**
   * @brief Get the reference position
   *
   * @param reference_position Vector3 Reference position
   */
  inline void get_reference_position(Vector3& reference_position) const {
    reference_position = reference_position_;
  }

  /**
   * @brief Get the reference position
   *
   * @return const Vector3& Reference position
   */
  inline const Vector3& get_reference_position() const { return reference_position_; }

  /**
   * @brief Get the reference velocity
   *
   * @param reference_velocity Vector3 Reference velocity
   */
  inline void get_reference_velocity(Vector3& reference_velocity) const {
    reference_velocity = reference_velocity_;
  }

  /**
   * @brief Get the reference velocity
   *
   * @return const Vector3& Reference velocity
   */
  inline const Vector3& get_reference_velocity() const { return reference_velocity_; }

  /**
   * @brief Get the reference acceleration
   *
   * @param reference_acceleration Vector3 Reference acceleration
   */
  inline void get_reference_acceleration(Vector3& reference_acceleration) const {
    reference_acceleration = reference_acceleration_;
  }

  /**
   * @brief Get the reference acceleration
   *
   * @return const Vector3& Reference acceleration
   */
  inline const Vector3& get_reference_acceleration() const { return reference_acceleration_; }

  /**
   * @brief Get the reference yaw
   *
   * @param reference_yaw Scalar Reference yaw
   */
  inline void get_reference_yaw(Scalar& reference_yaw) const { reference_yaw = reference_yaw_; }

  /**
   * @brief Get the reference yaw
   *
   * @return Scalar Reference yaw
   */
  inline Scalar get_reference_yaw() const { return reference_yaw_; }

  /**
   * @brief Get the reference yaw rate
   *
   * @param reference_yaw_rate Scalar Reference yaw rate
   */
  inline void get_reference_yaw_rate(Scalar& reference_yaw_rate) const {
    reference_yaw_rate = reference_yaw_rate_;
  }

  /**
   * @brief Get the reference yaw rate
   *
   * @return Scalar Reference yaw rate
   */
  inline Scalar get_reference_yaw_rate() const { return reference_yaw_rate_; }

  /**
   * @brief Get the actuation motors angular velocity
   *
   * @param actuation_motors_angular_velocity VectorN Actuation motors angular velocity
   */
  inline void get_actuation_motors_angular_velocity(
      VectorN& actuation_motors_angular_velocity) const {
    actuation_motors_angular_velocity = actuation_motors_angular_velocity_;
  }

  /**
   * @brief Get the actuation motors angular velocity
   *
   * @return const VectorN& Actuation motors angular velocity
   */
  inline const VectorN& get_actuation_motors_angular_velocity() const {
    return actuation_motors_angular_velocity_;
  }

  /**
   * @brief Get the external force
   *
   * @param external_force Vector3 External force
   */
  inline void get_external_force(Vector3& external_force) const {
    external_force = external_force_;
  }

  /**
   * @brief Get the external force
   *
   * @return const Vector3& External force
   */
  inline const Vector3& get_external_force() const { return external_force_; }

  /**
   * @brief Get the control mode
   *
   * @param control_mode ControlMode Control mode
   */
  inline void get_control_mode(ControlMode& control_mode) const { control_mode = control_mode_; }

  /**
   * @brief Get the control mode
   *
   * @return const ControlMode& Control mode
   */
  inline const ControlMode& get_control_mode() const { return control_mode_; }

  /**
   * @brief Get the yaw control mode
   *
   * @param yaw_control_mode YawControlMode Yaw control mode
   */
  inline void get_yaw_control_mode(YawControlMode& yaw_control_mode) const {
    yaw_control_mode = yaw_control_mode_;
  }

  /**
   * @brief Get the yaw control mode
   *
   * @return const YawControlMode& Yaw control mode
   */
  inline const YawControlMode& get_yaw_control_mode() const { return yaw_control_mode_; }
};
}  // namespace multirotor

#endif  // MULTIROTOR_SIMULATOR_MULTIROTOR_SIMULATOR_HPP_
