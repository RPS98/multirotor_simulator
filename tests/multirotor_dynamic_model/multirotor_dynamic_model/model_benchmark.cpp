/*!*******************************************************************************************
 *  \file       model_benchmark.hpp
 *  \brief      Model class benchmark.
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
#include "multirotor_dynamic_model/model.hpp"

namespace multirotor::model {

ModelParams<double, 4> get_model_x_config_params() {
  const double thrust_coefficient = 2.0;
  const double torque_coefficient = 1.0;
  const double x_dist             = 0.2;
  const double y_dist             = 0.3;
  const double min_speed          = 0.0;
  const double max_speed          = 100.0;
  const double time_constant      = 0.1;
  const double rotational_inertia = 0.5;

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
  return model_params;
}

static void BM_TEST_INIT(benchmark::State &state) {
  // Perform setup here
  ModelParams<double, 4> model_params = get_model_x_config_params();
  for (auto _ : state) {
    // This code gets timed
    Model<double, 4> model(model_params);
  }
}
BENCHMARK(BM_TEST_INIT)->Threads(1)->Repetitions(10);

static void BM_TEST_update_params(benchmark::State &state) {
  // Perform setup here
  ModelParams<double, 4> model_params = get_model_x_config_params();
  Model<double, 4> model(model_params);
  for (auto _ : state) {
    // This code gets timed
    model.update_params(model_params);
  }
}
BENCHMARK(BM_TEST_update_params)->Threads(1)->Repetitions(10);

static void BM_TEST_get_force_thrust_by_motors(benchmark::State &state) {
  // Perform setup here
  ModelParams<double, 4> model_params = get_model_x_config_params();
  Model<double, 4> model(model_params);
  Eigen::VectorXd motor_speeds = Eigen::VectorXd::Random(4);
  for (auto _ : state) {
    // This code gets timed
    model.get_force_thrust_by_motors(motor_speeds);
  }
}
BENCHMARK(BM_TEST_get_force_thrust_by_motors)->Threads(1)->Repetitions(10);

static void BM_TEST_get_force_vector_thrust_by_motors(benchmark::State &state) {
  // Perform setup here
  ModelParams<double, 4> model_params = get_model_x_config_params();
  Model<double, 4> model(model_params);
  Eigen::VectorXd motor_speeds = Eigen::VectorXd::Random(4);
  for (auto _ : state) {
    // This code gets timed
    model.get_force_vector_thrust_by_motors(motor_speeds);
  }
}
BENCHMARK(BM_TEST_get_force_vector_thrust_by_motors)->Threads(1)->Repetitions(10);

static void BM_TEST_get_torque_by_motors(benchmark::State &state) {
  // Perform setup here
  ModelParams<double, 4> model_params = get_model_x_config_params();
  Model<double, 4> model(model_params);
  Eigen::VectorXd motor_speeds = Eigen::VectorXd::Random(4);
  Eigen::VectorXd motor_accel  = Eigen::VectorXd::Random(4);
  for (auto _ : state) {
    // This code gets timed
    model.get_torque_by_motors(motor_speeds, motor_accel);
  }
}

BENCHMARK(BM_TEST_get_torque_by_motors)->Threads(1)->Repetitions(10);

}  // namespace multirotor::model

BENCHMARK_MAIN();
