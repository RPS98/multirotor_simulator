/*!*******************************************************************************************
 *  \file       trajectory_controller_benchmark.cpp
 *  \brief      Class benchmark
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

#include "multirotor_controllers/controllers/trajectory_controller.hpp"

namespace trajectory_controller {

pid_controller::PIDParams<> get_pid_params() {
  pid_controller::PIDParams pid_params;
  pid_params.Kp_gains = Eigen::Vector3d::Ones() * 3.0;
  pid_params.Ki_gains = Eigen::Vector3d::Ones() * 0.3;
  pid_params.Kd_gains = Eigen::Vector3d::Ones() * 0.9;

  pid_params.antiwindup_cte      = Eigen::Vector3d::Ones() * 10.0;
  pid_params.alpha               = Eigen::Vector3d::Ones() * 0.1;
  pid_params.reset_integral_flag = false;

  pid_params.proportional_saturation_flag = true;
  pid_params.upper_output_saturation      = Eigen::Vector3d::Zero();
  pid_params.lower_output_saturation      = Eigen::Vector3d::Zero();
  return pid_params;
}

static void BM_TEST_INIT(benchmark::State &state) {
  // Perform setup here
  // State
  Eigen::Vector3d current_position = Eigen::Vector3d::Zero();
  Eigen::Vector3d current_velocity = Eigen::Vector3d::Zero();

  // Reference
  Eigen::Vector3d desired_position     = Eigen::Vector3d::Ones();
  Eigen::Vector3d desired_velocity     = Eigen::Vector3d::Ones();
  Eigen::Vector3d desired_acceleration = Eigen::Vector3d::Ones();

  // Output
  Eigen::Vector3d desired_linear_acceleration = Eigen::Vector3d::Zero();

  // Time
  double dt = 0.001;

  // Controller
  TrajectoryControllerParams trajectory_controller_params;
  pid_controller::PIDParams pid_params       = get_pid_params();
  trajectory_controller_params.pid_params    = pid_params;
  TrajectoryController trajectory_controller = TrajectoryController(trajectory_controller_params);

  for (auto _ : state) {
    // This code gets timed
    desired_linear_acceleration = trajectory_controller.trajectory_to_linear_acceleration(
        current_position, current_velocity, desired_position, desired_velocity,
        desired_acceleration, dt);
  }
}
BENCHMARK(BM_TEST_INIT)->Threads(1)->Repetitions(10);

}  // namespace trajectory_controller

BENCHMARK_MAIN();
