/*
 * Copyright (c) 2024.  Jan-Hendrik Ewers
 * SPDX-License-Identifier: GPL-3.0-only
 */

#include <benchmark/benchmark.h>

#include "jdrones/transforms.h"

using namespace jdrones::data;

static void BM_euler_to_quat(benchmark::State& state)
{
  VEC3 rpy = Eigen::Vector3d::Random();

  for (auto _ : state)
  {
    jdrones::euler_to_quat(rpy);
  }
}
BENCHMARK(BM_euler_to_quat);

static void BM_quat_to_euler(benchmark::State& state)
{
  VEC4 quat = Eigen::Vector4d::Random();

  for (auto _ : state)
  {
    jdrones::quat_to_euler(quat);
  }
}
BENCHMARK(BM_quat_to_euler);

BENCHMARK_MAIN();