/*!*******************************************************************************************
 *  \file       utils_benchmark.cpp
 *  \brief      Benchmark
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
#include "multirotor_dynamic_model/common/utils.hpp"

// Benchmark clamp_vector function
static void BM_ClampVector(benchmark::State &state) {
  // Create a random Eigen vector for testing
  using VectorType  = Eigen::VectorXf;  // Change the type based on your use case
  VectorType vector = VectorType::Random(state.range(0));

  // Perform the benchmark
  for (auto _ : state) {
    multirotor::utils::clamp_vector(vector, 0.0f, 1.0f);  // Change min and max values accordingly
    benchmark::DoNotOptimize(vector);
  }
}
BENCHMARK(BM_ClampVector)->Threads(1)->DenseRange(1, 4, 1)->Repetitions(4);

// Benchmark get_quaternion_derivative function
static void BM_GetQuaternionDerivative(benchmark::State &state) {
  // Create a random Eigen quaternion and angular velocity vector for testing
  using QuaternionType = Eigen::Quaternionf;  // Change the type based on your use case
  using VectorType     = Eigen::Vector3f;     // Change the type based on your use case
  QuaternionType q     = QuaternionType::UnitRandom();
  VectorType omega     = VectorType::Random();

  // Perform the benchmark
  for (auto _ : state) {
    multirotor::utils::get_quaternion_derivative(q, omega);
    benchmark::DoNotOptimize(q);
  }
}
BENCHMARK(BM_GetQuaternionDerivative)->Threads(1)->Repetitions(10);

// Benchmark get_quaternion_integrate function
static void BM_GetQuaternionIntegrate(benchmark::State &state) {
  // Create a random Eigen quaternion and angular velocity vector for testing
  using QuaternionType = Eigen::Quaternionf;  // Change the type based on your use case
  using VectorType     = Eigen::Vector3f;     // Change the type based on your use case
  QuaternionType q     = QuaternionType::UnitRandom();
  VectorType omega     = VectorType::Random();
  double dt            = 0.01;  // Change the time step accordingly

  // Perform the benchmark
  for (auto _ : state) {
    multirotor::utils::get_quaternion_integrate(q, omega, dt);
    benchmark::DoNotOptimize(q);
  }
}
BENCHMARK(BM_GetQuaternionIntegrate)->Threads(1)->Repetitions(10);

// Benchmark squared_keep_sign function
static void BM_SquaredKeepSign(benchmark::State &state) {
  // Create a random Eigen vector for testing
  using VectorType  = Eigen::VectorXf;  // Change the type based on your use case
  VectorType vector = VectorType::Random(state.range(0));

  // Perform the benchmark
  for (auto _ : state) {
    multirotor::utils::squared_keep_sign(vector);
    benchmark::DoNotOptimize(vector);
  }
}
BENCHMARK(BM_SquaredKeepSign)->Threads(1)->DenseRange(1, 4, 1)->Repetitions(4);

// Benchmark sqrt_keep_sign function
static void BM_SqrtKeepSign(benchmark::State &state) {
  // Create a random Eigen vector for testing
  using VectorType  = Eigen::VectorXf;  // Change the type based on your use case
  VectorType vector = VectorType::Random(state.range(0));

  // Perform the benchmark
  for (auto _ : state) {
    multirotor::utils::sqrt_keep_sign(vector);
    benchmark::DoNotOptimize(vector);
  }
}
BENCHMARK(BM_SqrtKeepSign)->Threads(1)->DenseRange(1, 4, 1)->Repetitions(4);

BENCHMARK_MAIN();
