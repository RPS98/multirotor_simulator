/*!*******************************************************************************************
 *  \file       acro_controller_benchmark.cpp
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

#include "multirotor_controllers/controllers/acro_controller.hpp"

namespace acro_controller {

static void BM_TEST_INIT(benchmark::State &state) {
  // Perform setup here
  // State
  Eigen::Matrix3d current_orientation = Eigen::Matrix3d::Identity();

  // Reference
  double desired_yaw                   = 0.0;
  Eigen::Vector3d desired_acceleration = Eigen::Vector3d::Ones();

  Eigen::Vector3d desired_thrust           = Eigen::Vector3d::Zero();
  Eigen::Vector3d desired_angular_velocity = Eigen::Vector3d::Zero();

  // Controller
  AcroControllerParams acro_controller_params;
  acro_controller_params.vehicle_mass = 1.0;
  acro_controller_params.kp_rot       = Eigen::Matrix3d::Identity();
  AcroController acro_controller      = AcroController(acro_controller_params);

  for (auto _ : state) {
    // This code gets timed
    desired_thrust           = acro_controller.acceleration_to_thrust(desired_acceleration);
    desired_angular_velocity = acro_controller.yaw_angle_to_angular_velocity(
        current_orientation, desired_yaw, desired_thrust);
  }
}
BENCHMARK(BM_TEST_INIT)->Threads(1)->Repetitions(10);

}  // namespace acro_controller

BENCHMARK_MAIN();
