/*!*******************************************************************************************
 *  \file       indi_controller_benchmark.hpp
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

#include "multirotor_controllers/controllers/indi_controller.hpp"

namespace indi_controller {

Eigen::Matrix3d get_vehicle_inertia() {
  // Inertia:
  // 0.0049      0      0
  //     0 0.0049      0
  //     0      0 0.0069
  return Eigen::Vector3d(0.0049, 0.0049, 0.0069).asDiagonal();
}

Eigen::Matrix<double, 4, 6> get_mixer_matrix_inverse(int motors_frame_type = 0) {
  Eigen::Matrix<double, 4, 6> mixer_matrix_inverse = Eigen::Matrix<double, 4, 6>::Zero();
  if (motors_frame_type == 0) {
    // Mixer matrix inverse:
    //          0            0       130890 -1.29771e-10 -3.27225e+06       961538
    //          0            0       130890  3.27225e+06  1.29771e-10      -961538
    //          0            0       130890  1.29771e-10  3.27225e+06       961538
    //          0            0       130890 -3.27225e+06 -1.29771e-10      -961538
    mixer_matrix_inverse << 0.0, 0.0, 130890.0, -1.29771e-10, -3.27225e+06, 961538.0, 0.0, 0.0,
        130890.0, 3.27225e+06, 1.29771e-10, -961538.0, 0.0, 0.0, 130890.0, 1.29771e-10, 3.27225e+06,
        961538.0, 0.0, 0.0, 130890.0, -3.27225e+06, -1.29771e-10, -961538.0;

  } else {
    // Mixer matrix inverse:
    //        0            0       130890 -1.63613e+06 -1.63613e+06       961538
    //        0            0       130890  1.63613e+06 -1.63613e+06      -961538
    //        0            0       130890  1.63613e+06  1.63613e+06       961538
    //        0            0       130890 -1.63613e+06  1.63613e+06      -961538
    mixer_matrix_inverse << 0.0, 0.0, 130890.0, -1.63613e+06, -1.63613e+06, 961538.0, 0.0, 0.0,
        130890.0, 1.63613e+06, -1.63613e+06, -961538.0, 0.0, 0.0, 130890.0, 1.63613e+06,
        1.63613e+06, 961538.0, 0.0, 0.0, 130890.0, -1.63613e+06, 1.63613e+06, -961538.0;
  }
  return mixer_matrix_inverse;
}

pid_controller::PIDParams<> get_pid_params() {
  pid_controller::PIDParams pid_params;
  pid_params.Kd_gains = Eigen::Vector3d::Ones() * 0.9;
  pid_params.Ki_gains = Eigen::Vector3d::Ones() * 3.0;
  pid_params.Kp_gains = Eigen::Vector3d::Ones() * 0.3;

  pid_params.antiwindup_cte      = Eigen::Vector3d::Ones() * 10.0;
  pid_params.alpha               = Eigen::Vector3d::Ones() * 0.1;
  pid_params.reset_integral_flag = false;

  pid_params.proportional_saturation_flag = true;
  pid_params.upper_output_saturation      = Eigen::Vector3d::Zero();
  pid_params.lower_output_saturation      = Eigen::Vector3d::Zero();
  return pid_params;
}

// Benchmark acro_to_motor_angular_velocity function
static void BM_acro_to_motor_angular_velocity(benchmark::State &state) {
  pid_controller::PIDParams pid_params = get_pid_params();
  IndiController indi_controller =
      IndiController(get_vehicle_inertia(), get_mixer_matrix_inverse(), pid_params);

  // Input move forward
  double thrust                            = 1.0;
  Eigen::Vector3d desired_angular_velocity = Eigen::Vector3d::Zero();
  Eigen::Vector3d current_angular_velocity = Eigen::Vector3d::Zero();
  double dt                                = 0.01;

  // Output
  Eigen::Vector4d output_motor_angular_velocity;

  // Perform the benchmark
  for (auto _ : state) {
    output_motor_angular_velocity = indi_controller.acro_to_motor_angular_velocity(
        current_angular_velocity, thrust, desired_angular_velocity, dt);
  }
}
BENCHMARK(BM_acro_to_motor_angular_velocity)->Threads(1)->Repetitions(10);

}  // namespace indi_controller

BENCHMARK_MAIN();
