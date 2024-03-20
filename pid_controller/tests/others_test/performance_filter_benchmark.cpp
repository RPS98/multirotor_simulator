/*!*******************************************************************************************
 *  \file       performance_filter_benchmark.hpp
 *  \brief      Filter performance benchmark
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

Vector3d filtered = Vector3d::Ones();
double alpha_1    = 1.0;
Matrix3d alpha_2  = Matrix3d::Identity();
Vector3d alpha_3  = Vector3d::Ones();

Vector3d computeFilter1(Vector3d &new_value) {
  return filtered = alpha_1 * new_value + (1.0 - alpha_1) * filtered;
}

Vector3d computeFilter2(Vector3d &new_value) {
  return filtered = alpha_2 * new_value + (Matrix3d::Identity() - alpha_2) * filtered;
}

Vector3d computeFilter3(Vector3d &new_value) {
  return filtered =
             alpha_3.cwiseProduct(new_value) + (Vector3d::Ones() - alpha_3).cwiseProduct(filtered);
}

static void BM_TEST_FILTER1(benchmark::State &state) {
  Vector3d value = Vector3d::Ones();
  filtered       = Vector3d::Ones();

  for (auto _ : state) {
    computeFilter1(value);
  }
}
BENCHMARK(BM_TEST_FILTER1)->Threads(1)->Repetitions(20);

static void BM_TEST_FILTER2(benchmark::State &state) {
  Vector3d value = Vector3d::Ones();
  filtered       = Vector3d::Ones();
  for (auto _ : state) {
    computeFilter2(value);
  }
}
BENCHMARK(BM_TEST_FILTER2)->Threads(1)->Repetitions(20);

static void BM_TEST_FILTER3(benchmark::State &state) {
  Vector3d value = Vector3d::Ones();
  filtered       = Vector3d::Ones();
  for (auto _ : state) {
    computeFilter3(value);
  }
}
BENCHMARK(BM_TEST_FILTER3)->Threads(1)->Repetitions(20);

BENCHMARK_MAIN();
