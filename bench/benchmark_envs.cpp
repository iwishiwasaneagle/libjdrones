/*
 * Copyright (c) 2024.  Jan-Hendrik Ewers
 * SPDX-License-Identifier: GPL-3.0-only
 */

#include <benchmark/benchmark.h>

#include "jdrones/dynamics.h"
#include "jdrones/envs.h"

static void BM_LinearDynamicsDroneEnv(benchmark::State& state)
{
  jdrones::dynamics::LinearDynamicModelDroneEnv env(0.01);
  VEC4 u{ 10, 10, 10, 10 };

  for (auto _ : state)
  {
    env.step(u);
  }
}
BENCHMARK(BM_LinearDynamicsDroneEnv)->Threads(1)->Threads(20);

static void BM_NonLinearDynamicsDroneEnv(benchmark::State& state)
{
  jdrones::dynamics::NonlinearDynamicModelDroneEnv env(0.01);
  VEC4 u{ 10, 10, 10, 10 };

  for (auto _ : state)
  {
    env.step(u);
  }
}
BENCHMARK(BM_NonLinearDynamicsDroneEnv)->Threads(1)->Threads(20);

static void BM_LQRDroneEnv(benchmark::State& state)
{
  jdrones::envs::LQRDroneEnv env(0.01);
  State setpoint;
  setpoint.set_pos({ 10, 10, 10 });

  for (auto _ : state)
  {
    env.reset();
    env.step(setpoint);
  }
}
BENCHMARK(BM_LQRDroneEnv)->Threads(1)->Threads(20);

static void BM_FifthOrderPolyDroneEnv(benchmark::State& state)
{
  jdrones::envs::FifthOrderPolyPositionDroneEnv env(0.01);

  for (auto _ : state)
  {
    env.reset();
    env.step({ 10, 10, 10 });
  }
}
BENCHMARK(BM_FifthOrderPolyDroneEnv)->Threads(1)->Threads(20);

static void BM_OptimalFifthOrderPolyDroneEnv(benchmark::State& state)
{
  jdrones::envs::OptimalFifthOrderPolyPositionDroneEnv env(0.01);

  for (auto _ : state)
  {
    env.reset();
    env.step({ 10, 10, 10 });
  }
}
BENCHMARK(BM_OptimalFifthOrderPolyDroneEnv)->Threads(1)->Threads(20);

BENCHMARK_MAIN();