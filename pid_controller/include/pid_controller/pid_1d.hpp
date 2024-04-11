/*!*******************************************************************************************
 *  \file       pid_1.hpp
 *  \brief      PID 1D Controller definition
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

#ifndef PID_CONTROLLER_PID_1D_HPP_
#define PID_CONTROLLER_PID_1D_HPP_

#include <math.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <iostream>
#include <limits>

namespace pid_1d_controller {

template <typename P = double>
struct PIDParams {
  static_assert(std::is_floating_point<P>::value,
                "MotorParams must be used with a floating-point type");

  using Scalar = P;

  // PID gains
  Scalar Kp_gains = 0.0;  // Proportional gains
  Scalar Ki_gains = 0.0;  // Integral gains
  Scalar Kd_gains = 0.0;  // Derivative gains

  // PID params
  Scalar antiwindup_cte    = 0.0;    // Integral anti-windup
  Scalar alpha             = 1.0;    // Derivative filter
  bool reset_integral_flag = false;  // Reset integral flag when error sign changes

  // PID Output saturation
  Scalar upper_output_saturation = 0.0;  // Upper output saturation
  Scalar lower_output_saturation = 0.0;  // Lower output saturation
};

/**
 * @brief PID Controller
 *
 * PID 1 dimension controller. It can be used to control any 1D system.
 *
 * @tparam P Precision
 */
template <typename P = double>
class PID {
  static_assert(std::is_floating_point<P>::value,
                "MotorParams must be used with a floating-point type");
  using Scalar = P;

public:
  /**
   * @brief Construct a new pid
   *
   * @param verbose Verbosity flag. Default: false
   */
  explicit PID(const PIDParams<P> &pid_params = PIDParams<P>(), const bool &verbose = false)
      : verbose_(verbose) {
    update_params(pid_params);
    reset_controller();
  }

  ~PID() {}

private:
  bool verbose_ = false;  // Verbosity flag

  // PID gains
  Scalar Kp_ = 0.0;
  Scalar Ki_ = 0.0;
  Scalar Kd_ = 0.0;

  // PID params
  Scalar antiwindup_cte_    = 0.0;    // Integral anti-windup
  Scalar alpha_             = 1.0;    // Derivative filter
  bool reset_integral_flag_ = false;  // Reset integral flag when error sign changes

  // PID Output saturation
  bool saturation_flag_           = false;  // Output saturation flag
  Scalar upper_output_saturation_ = 0.0;
  Scalar lower_output_saturation_ = 0.0;

  // PID state
  bool first_run_                 = true;  // First run flag
  Scalar integral_accum_error_    = 0.0;   // Integral accumulator error
  Scalar filtered_derivate_error_ = 0.0;   // Filtered derivative error

  // Error and output storage (for debugging purposes)
  Scalar proportional_error_              = 0.0;  // Proportional error
  Scalar derivative_error_                = 0.0;  // Derivative error
  Scalar proportional_error_contribution_ = 0.0;  // Proportional error contribution
  Scalar integral_error_contribution_     = 0.0;  // Integral error contribution
  Scalar derivate_error_contribution_     = 0.0;  // Derivative error contribution
  Scalar output_                          = 0.0;  // Output

public:
  // Public methods

  /**
   * @brief Update the PID controller with pid params
   *
   * @param params PIDParams struct
   */
  void update_params(const PIDParams<P> &params) {
    set_gains(params.Kp_gains, params.Ki_gains, params.Kd_gains);
    set_anti_windup(params.antiwindup_cte);
    set_alpha(params.alpha);
    set_reset_integral_saturation_flag(params.reset_integral_flag);

    if ((params.upper_output_saturation != 0.0) || (params.lower_output_saturation != 0.0)) {
      set_output_saturation(params.upper_output_saturation, params.lower_output_saturation);
    } else {
      disable_output_saturation();
    }
  }

  /**
   * @brief Reset the controller
   *
   * Reset the integral error and the saturation
   */
  inline void reset_controller() { first_run_ = true; }

  /**
   * @brief Set the output saturation
   *
   * @param upper_saturation Upper saturation
   * @param lower_saturation Lower saturation
   */
  void set_output_saturation(const Scalar upper_saturation, const Scalar lower_saturation) {
    // Check if different between upper and lower saturation is greater than epsilon
    if (std::abs(upper_saturation - lower_saturation) < std::numeric_limits<Scalar>::epsilon()) {
      std::cerr << "Upper and lower saturation are equal. Saturation is disabled" << std::endl;
      disable_output_saturation();
      return;
    }
    // Check if upper saturation is greater than lower saturation
    if (upper_saturation < lower_saturation) {
      std::cerr << "Upper saturation is lower than lower saturation. Saturation is disabled"
                << std::endl;
      disable_output_saturation();
      return;
    }

    upper_output_saturation_ = upper_saturation;
    lower_output_saturation_ = lower_saturation;
    saturation_flag_         = true;
  }

  /**
   * @brief Disable the output saturation
   *
   * Disable the output saturation. The output is not limited by the saturation
   * limits. To enable the output saturation, use the set_output_saturation
   * method.
   *
   * @param saturation_flag Saturation flag
   */
  inline void disable_output_saturation() { saturation_flag_ = false; }

  /**
   * @brief Get the proportional error
   *
   * @param state Current state
   * @param reference Reference state
   * @return Scalar Error
   */
  static inline Scalar get_error(const Scalar state, const Scalar reference) {
    // Compute proportional error
    return reference - state;
  }

  /**
   * @brief Get the proportional and derivative error
   *
   * @param state State
   * @param reference Reference
   * @param state_dot State derivative
   * @param reference_dot Reference derivative
   * @param proportional_error Output proportional error
   * @param derivative_error Output derivative error
   */
  static inline void get_error(const Scalar state,
                               const Scalar reference,
                               const Scalar state_dot,
                               const Scalar reference_dot,
                               Scalar &proportional_error,
                               Scalar &derivative_error) {
    // Compute proportional error
    proportional_error = reference - state;

    // Compute the derivate error
    derivative_error = reference_dot - state_dot;
  }

  /**
   * @brief Process the PID controller
   *
   * @param dt Time step
   * @param proportional_error Proportional error
   * @return PID output
   */
  Scalar compute_control(const Scalar dt, const Scalar proportional_error) {
    // Initialize values for the integral and derivative contributions
    if (first_run_) {
      first_run_               = false;
      integral_accum_error_    = 0.0;
      proportional_error_      = proportional_error;
      filtered_derivate_error_ = 0.0;
    }

    // Compute the proportional contribution
    proportional_error_contribution_ = Kp_ * proportional_error;

    // Compute de integral contribution (position integrate)
    integral_error_contribution_ = compute_integral_contribution(dt, proportional_error);

    // Compute the derivate contribution
    derivate_error_contribution_ =
        compute_derivative_contribution_by_deriving(dt, proportional_error);

    // Compute output velocity
    output_ = proportional_error_contribution_ + integral_error_contribution_ +
              derivate_error_contribution_;

    if (saturation_flag_) {
      output_ = saturate_output(output_, upper_output_saturation_, lower_output_saturation_);
    }

    // Update last proportional error
    proportional_error_ = proportional_error;
    return output_;
  }

  /**
   * @brief Process the PID controller
   *
   * @param dt = Time step
   * @param proportional_error = Proportional error
   * @param derivative_error = Derivative error
   * @return PID output
   */
  Scalar compute_control(const Scalar dt,
                         const Scalar proportional_error,
                         const Scalar derivative_error) {
    // Initialize values for the integral and derivative contributions
    if (first_run_) {
      first_run_               = false;
      integral_accum_error_    = 0.0;
      proportional_error_      = proportional_error;
      filtered_derivate_error_ = 0.0;
    }

    // Compute the proportional contribution
    proportional_error_contribution_ = Kp_ * proportional_error;

    // Compute de integral contribution (position integrate)
    integral_error_contribution_ = compute_integral_contribution(dt, proportional_error);

    // Compute the derivate contribution
    derivate_error_contribution_ = compute_derivative_contribution(derivative_error);

    // Compute output velocity
    output_ = proportional_error_contribution_ + integral_error_contribution_ +
              derivate_error_contribution_;

    if (saturation_flag_) {
      output_ = saturate_output(output_, upper_output_saturation_, lower_output_saturation_);
    }

    // Update last proportional error
    proportional_error_ = proportional_error;
    derivative_error_   = derivative_error;
    return output_;
  }

  /**
   * @brief Saturation function
   *
   * If the output is greater than the upper limit, the output is saturated to
   * the upper limit. If the output is lower than the lower limit, the output is
   * saturated to the lower limit.
   *
   * @param output Value to saturate
   * @param upper_limits Upper limits vector
   * @param lower_limits Lower limits vector
   * @return Saturated value
   */
  static inline Scalar saturate_output(const Scalar output,
                                       const Scalar upper_limits,
                                       const Scalar lower_limits) {
    // With std::clamp
    return std::clamp(output, lower_limits, upper_limits);
  }

  // Getters and setters

  /**
   * @brief Get the params
   *
   * @return PIDParams<P> PID parameters
   */
  PIDParams<P> get_params() const {
    PIDParams<P> params;
    get_gains(params.Kp_gains, params.Ki_gains, params.Kd_gains);
    params.antiwindup_cte      = get_anti_windup();
    params.alpha               = get_alpha();
    params.reset_integral_flag = get_reset_integral_saturation_flag();
    get_saturation_limits(params.upper_output_saturation, params.lower_output_saturation);
    return params;
  }

  /**
   * @brief Set the Gains of the controller
   *
   * @param kp Proportional gain
   * @param ki Integral gain
   * @param kd Derivative gain
   */
  inline void set_gains(Scalar kp, Scalar ki, Scalar kd) {
    Kp_ = kp;
    Ki_ = ki;
    Kd_ = kd;
  }

  /**
   * @brief Get the gains
   *
   * @param kp Proportional gain
   * @param ki Integral gain
   * @param kd Derivative gain
   */
  inline void get_gains(Scalar &kp, Scalar &ki, Scalar &kd) const {
    kp = Kp_;
    ki = Ki_;
    kd = Kd_;
  }

  /**
   * @brief Set the Proportional Gain of the controller
   *
   * @param kp Proportional gain
   */
  inline void set_kp(Scalar kp) { Kp_ = kp; }

  /**
   * @brief Get the Proportional Gain of the controller
   *
   * @return Scalar Proportional gain
   */
  inline Scalar get_kp() const { return Kp_; }

  /**
   * @brief Set the Integral Gain of the controller
   *
   * @param ki Integral gain
   */
  inline void set_ki(Scalar ki) { Ki_ = ki; }

  /**
   * @brief Get the Integral Gain of the controller
   *
   * @return Scalar Integral gain
   */
  inline Scalar get_ki() const { return Ki_; }

  /**
   * @brief Set the Derivative Gain of the controller
   *
   * @param kd Derivative gain
   */
  inline void set_kd(Scalar kd) { Kd_ = kd; }

  /**
   * @brief Get the Derivative Gain of the controller
   *
   * @return Scalar Derivative gain
   */
  inline Scalar get_kd() const { return Kd_; }

  /**
   * @brief Set the Anti Windup of the controller
   *
   * @param anti_windup Anti windup
   */
  inline void set_anti_windup(Scalar anti_windup) { antiwindup_cte_ = anti_windup; }

  /**
   * @brief Get the Anti Windup of the controller
   *
   * @return Scalar Anti windup
   */
  inline Scalar get_anti_windup() const { return antiwindup_cte_; }

  /**
   * @brief Set the Alpha of the controller
   *
   * @param alpha Alpha
   */
  inline void set_alpha(Scalar alpha) { alpha_ = alpha; }

  /**
   * @brief Get the Alpha of the controller
   *
   * @return Scalar Alpha
   */
  inline Scalar get_alpha() const { return alpha_; }

  /**
   * @brief Set the Reset Integral Saturation Flag of the controller
   *
   * If the flag is true, the integral contribution is reset to zero when the
   * integral error is grater than the anti windup and the sign of the integral
   * error is different from the sign of the proportional error.
   *
   * @param reset_integral_flag Reset integral saturation flag
   */
  inline void set_reset_integral_saturation_flag(bool reset_integral_flag) {
    reset_integral_flag_ = reset_integral_flag;
  }

  /**
   * @brief Get the Reset Integral Saturation Flag of the controller
   *
   * If the flag is true, the integral contribution is reset to zero when the
   * integral error is grater than the anti windup and the sign of the integral
   * error is different from the sign of the proportional error.
   *
   * @return Scalarrue Reset integral saturation flag is enabled
   * @return false Reset integral saturation flag is disabled
   */
  inline bool get_reset_integral_saturation_flag() const { return reset_integral_flag_; }

  /**
   * @brief Get the Saturation Limits of the controller
   *
   * @param upper_limit Upper limit
   * @param lower_limit Lower limit
   */
  inline void get_saturation_limits(Scalar &upper_limit, Scalar &lower_limit) const {
    upper_limit = upper_output_saturation_;
    lower_limit = lower_output_saturation_;
  }

  /**
   * @brief Get the output saturation flag
   *
   * @return Scalarrue Saturation is enabled
   * @return false Saturation is disabled
   */
  inline bool get_output_saturation_flag() const { return saturation_flag_; }

  /**
   * @brief Get the proportional error
   *
   * @return Scalar Proportional error
   */
  inline Scalar get_proportional_error() const { return proportional_error_; }

  /**
   * @brief Get the derivative error
   *
   * @return Scalar Derivative error
   */
  inline Scalar get_derivative_error() const { return derivative_error_; }

  /**
   * @brief Get the proportional error contribution
   *
   * @return Scalar Proportional error contribution
   */
  inline Scalar get_proportional_error_contribution() const {
    return proportional_error_contribution_;
  }

  /**
   * @brief Get the integral error contribution
   *
   * @return Scalar Integral error contribution
   */
  inline Scalar get_integral_error_contribution() const { return integral_error_contribution_; }

  /**
   * @brief Get the derivative error contribution
   *
   * @return Scalar Derivative error contribution
   */
  inline Scalar get_derivative_error_contribution() const { return derivate_error_contribution_; }

  /**
   * @brief Get the output
   *
   * @return Scalar Output
   */
  inline Scalar get_output() const { return output_; }

protected:
  /**
   * @brief Compute the integral contribution of the controller
   *
   * @param dt Delta time (s)
   * @param proportional_error Proportional error
   * @return Scalar Integral contribution
   */
  Scalar compute_integral_contribution(const Scalar dt, const Scalar proportional_error) {
    // If sing of the error changes and the integrator is saturated, reset the
    // integral for each axis
    if (reset_integral_flag_ != 0) {
      if (std::abs(integral_accum_error_) > antiwindup_cte_) {
        if (std::signbit(integral_accum_error_) != std::signbit(proportional_error)) {
          integral_accum_error_ = 0.0;
        }
      }
    }

    // Update de acumulated error
    integral_accum_error_ += proportional_error * dt;

    // Compute anti-windup. Limit integral contribution
    if (antiwindup_cte_ != 0.0) {
      integral_accum_error_ =
          saturate_output(integral_accum_error_, antiwindup_cte_, -1.0 * antiwindup_cte_);
    }

    // Compute de integral contribution
    Scalar integral_error_contribution = Ki_ * integral_accum_error_;
    return integral_error_contribution;
  }

  /**
   * @brief Compute the derivative contribution of the controller
   *
   * @param dt Delta time (s)
   * @param proportional_error Proportional error
   * @return Derivative contribution
   */
  Scalar compute_derivative_contribution_by_deriving(const Scalar dt,
                                                     const Scalar proportional_error) {
    // Compute the derivative contribution of the error filtered with a first
    // order filter
    Scalar derivate_proportional_error_increment = (proportional_error - proportional_error_) / dt;

    filtered_derivate_error_ =
        alpha_ * derivate_proportional_error_increment + (1.0 - alpha_) * filtered_derivate_error_;

    // Compute the derivate contribution
    Scalar derivate_error_contribution = compute_derivative_contribution(filtered_derivate_error_);
    return derivate_error_contribution;
  }

  /**
   * @brief Compute the derivative contribution of the controller
   *
   * For controllers with derivative feedback, the derivative contribution is
   * computed using the state and reference derivatives.
   *
   * @param state_dot State derivative
   * @param reference_dot Reference derivative
   * @return Derivative contribution
   */
  inline Scalar compute_derivative_contribution(const Scalar derivate_error) {
    // Compute the derivate contribution
    Scalar derivate_error_contribution = Kd_ * derivate_error;
    return derivate_error_contribution;
  }
};

}  // namespace pid_1d_controller

#endif  // PID_CONTROLLER_PID_1D_HPP_
