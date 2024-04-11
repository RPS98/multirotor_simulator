/*!*******************************************************************************************
 *  \file       test_performance_ResetIntegral_benchmark.hpp
 *  \brief      PID Controller benchmark
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

using Vector3d = Eigen::Vector3d;
using Matrix3d = Eigen::Matrix3d;

bool reset_integral_flag_     = true;
Vector3d integral_accum_error = 5 * Vector3d::Ones();
Vector3d antiwindup_cte_      = Vector3d::Ones();
double antiwindup_cte_d       = 1.0;
Vector3d _proportional_error  = -1.0 * Vector3d::Ones();

Vector3d resetIntegral1(Vector3d &integral_accum_error_, const bool &proportional_limitation) {
  if (reset_integral_flag_) {
    for (int j = 0; j < 3; j++) {
      if (std::abs(integral_accum_error_[j]) > antiwindup_cte_[j]) {
        if (std::signbit(integral_accum_error_[j]) != std::signbit(_proportional_error[j])) {
          integral_accum_error_[j] = 0.0f;
        }
      }
    }
  }
  return integral_accum_error_;
}

Vector3d resetIntegral2(Vector3d &integral_accum_error_, const bool &proportional_limitation) {
  if (reset_integral_flag_) {
    for (int j = 0; j < 3; j++) {
      if (std::abs(integral_accum_error_[j]) > antiwindup_cte_[j]) {
        if (std::signbit(integral_accum_error_[j]) != std::signbit(_proportional_error[j])) {
          integral_accum_error_[j] = 0.0f;
        }
      }
    }
  }
  return integral_accum_error_;
}

static void BM_TEST_RESETINTEGRAL1(benchmark::State &state) {
  bool reset_integral_flag_     = true;
  Vector3d integral_accum_error = 5 * Vector3d::Ones();
  Vector3d antiwindup_cte_      = Vector3d::Ones();
  double antiwindup_cte_d       = 1.0;
  Vector3d _proportional_error  = -1.0 * Vector3d::Ones();
  for (auto _ : state) {
    Vector3d output = resetIntegral1(integral_accum_error, reset_integral_flag_);
  }
}
BENCHMARK(BM_TEST_RESETINTEGRAL1)->Threads(1)->Repetitions(20);

static void BM_TEST_RESETINTEGRAL2(benchmark::State &state) {
  bool reset_integral_flag_     = true;
  Vector3d integral_accum_error = 5 * Vector3d::Ones();
  Vector3d antiwindup_cte_      = Vector3d::Ones();
  double antiwindup_cte_d       = 1.0;
  Vector3d _proportional_error  = -1.0 * Vector3d::Ones();
  for (auto _ : state) {
    Vector3d output = resetIntegral2(integral_accum_error, reset_integral_flag_);
  }
}
BENCHMARK(BM_TEST_RESETINTEGRAL2)->Threads(1)->Repetitions(20);

BENCHMARK_MAIN();
