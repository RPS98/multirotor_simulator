/*!*******************************************************************************************
 *  \file       dynamics_benchmark.cpp
 *  \brief      Dynamics benchmark
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

#include <benchmark/benchmark.h>
#include <exception>
#include <memory>
#include "multirotor_dynamic_model/dynamics.hpp"

template <typename P = double, int num_rotors = 4>
using State = multirotor::state::State<P, num_rotors>;
template <typename P = double>
using MotorParams = multirotor::model::MotorParams<P>;
template <typename P = double, int num_rotors = 4>
using ModelParams = multirotor::model::ModelParams<P, num_rotors>;
template <typename P = double, int num_rotors = 4>
using Model = multirotor::model::Model<P, num_rotors>;
template <typename P = double, int num_rotors = 4>
using Dynamics = multirotor::dynamics::Dynamics<P, num_rotors>;

Model<double, 4> get_model() {
  const double thrust_coefficient = 1.91e-6;
  const double torque_coefficient = 2.6e-7;
  const double x_dist             = 0.08;
  const double y_dist             = 0.08;
  const double min_speed          = 0.0;
  const double max_speed          = 2200.0;
  const double time_constant      = 0.02;
  const double rotational_inertia = 6.62e-6;

  std::vector<MotorParams<double>> motors = Model<double, 4>::create_quadrotor_x_config(
      thrust_coefficient, torque_coefficient, x_dist, y_dist, min_speed, max_speed, time_constant,
      rotational_inertia);

  ModelParams<double, 4> model_params;
  // Motor characteristics
  model_params.motors_params = motors;
  // Vehicle characteristics
  model_params.vehicle_mass    = 1.0;
  model_params.vehicle_inertia = Eigen::Vector3d(0.0049, 0.0049, 0.0069).asDiagonal();
  // Aerodynamic characteristics
  model_params.vehicle_aero_moment_coefficient = Eigen::Vector3d(0.003, 0.003, 0.003).asDiagonal();
  model_params.vehicle_drag_coefficient        = 0.1;
  // Environment characteristics
  model_params.gravity                               = Eigen::Vector3d(0.0, 0.0, -9.81);
  model_params.force_process_noise_auto_correlation  = 5.00e-5;
  model_params.moment_process_noise_auto_correlation = 1.25e-7;

  // Create the model
  Model<double, 4> model = Model<double, 4>(model_params);
  return model;
}

static void BM_TEST_PROCESS_EULER_EXPLICIT(benchmark::State &bm_state) {
  // Perform setup here
  Model<double, 4> model = get_model();
  State<double, 4> state = State<double, 4>();

  Dynamics<double, 4> dynamics(model, state);

  const double max_speed = model.get_motors()[0].max_speed;

  Eigen::Vector4d actuation_angular_velocity = Eigen::Vector4d::Ones() * max_speed;
  double dt                                  = 0.001;

  for (auto _ : bm_state) {
    // This code gets timed
    dynamics.process_euler_explicit(actuation_angular_velocity, dt);
  }
}
BENCHMARK(BM_TEST_PROCESS_EULER_EXPLICIT)->Threads(1)->Repetitions(10);

static void BM_TEST_GET_STATE(benchmark::State &bm_state) {
  // Perform setup here
  Model<double, 4> model = get_model();
  State<double, 4> state = State<double, 4>();

  Dynamics<double, 4> dynamics(model, state);

  const double max_speed = model.get_motors()[0].max_speed;

  Eigen::Vector4d actuation_angular_velocity = Eigen::Vector4d::Ones() * max_speed;
  double dt                                  = 0.001;

  dynamics.process_euler_explicit(actuation_angular_velocity, dt);
  for (auto _ : bm_state) {
    // This code gets timed
    state = dynamics.get_state();
  }
}
BENCHMARK(BM_TEST_GET_STATE)->Threads(1)->Repetitions(10);

static void BM_TEST_GET_STATE_CONST(benchmark::State &bm_state) {
  // Perform setup here
  Model<double, 4> model = get_model();
  State<double, 4> state = State<double, 4>();

  Dynamics<double, 4> dynamics(model, state);

  const double max_speed = model.get_motors()[0].max_speed;

  Eigen::Vector4d actuation_angular_velocity = Eigen::Vector4d::Ones() * max_speed;
  double dt                                  = 0.001;

  dynamics.process_euler_explicit(actuation_angular_velocity, dt);
  for (auto _ : bm_state) {
    // This code gets timed
    const State<double, 4> output_state = dynamics.get_state();
  }
}
BENCHMARK(BM_TEST_GET_STATE_CONST)->Threads(1)->Repetitions(10);

BENCHMARK_MAIN();
