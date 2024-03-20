/*!*******************************************************************************************
 *  \file       pid_benchmark.hpp
 *  \brief      PID performance benchmark
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
#include <math.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <exception>
#include <iostream>

#include "pid_controller/pid.hpp"

namespace pid_controller {

using Vector = Eigen::Vector3d;
using Matrix = Eigen::Matrix3d;

static void BM_TEST(benchmark::State &b_state) {
  PIDParams pid_params;
  pid_params.Kp_gains                     = Vector::Ones();
  pid_params.Ki_gains                     = Vector::Ones();
  pid_params.Kd_gains                     = Vector::Ones();
  pid_params.antiwindup_cte               = Vector::Ones();
  pid_params.alpha                        = Vector::Ones();
  pid_params.reset_integral_flag          = false;
  pid_params.proportional_saturation_flag = false;
  pid_params.upper_output_saturation      = Vector::Ones() * 2.0;
  pid_params.lower_output_saturation      = Vector::Ones() * -2.0;

  PID pid(pid_params);

  Vector state     = Vector::Ones();
  Vector reference = Vector::Ones() * 2.0;
  Vector error     = Vector::Zero();
  Vector output    = Vector::Zero();
  double dt        = 0.01;

  for (auto _ : b_state) {
    error  = pid.get_error(state, reference);
    output = pid.compute_control(dt, error);
  }
}
BENCHMARK(BM_TEST)->Threads(1)->Repetitions(10);

}  // namespace pid_controller

BENCHMARK_MAIN();
