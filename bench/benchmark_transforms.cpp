/*
 * Copyright (c) 2024.  Jan-Hendrik Ewers
 * SPDX-License-Identifier: GPL-3.0-only
 */

#include <benchmark/benchmark.h>

#include "jdrones/transforms.h"

static void BM_euler_to_quat(benchmark::State& state)
{
  Eigen::Vector3d rpy = Eigen::Vector3d::Random();

  for (auto _ : state)
  {
    jdrones::euler_to_quat(rpy);
  }
}
BENCHMARK(BM_euler_to_quat);

static void BM_quat_to_euler(benchmark::State& state)
{
  Eigen::Vector4d quat = Eigen::Vector4d::Random();

  for (auto _ : state)
  {
    jdrones::quat_to_euler(quat);
  }
}
BENCHMARK(BM_quat_to_euler);

BENCHMARK_MAIN();