/*
 * Copyright (c) 2024.  Jan-Hendrik Ewers
 * SPDX-License-Identifier: GPL-3.0-only
 */

#include <benchmark/benchmark.h>

#include "jdrones/data.h"

static void BM_x_to_state(benchmark::State& state)
{
  jdrones::data::X x = jdrones::data::X::Random();

  for (auto _ : state)
  {
    jdrones::data::x_to_state(x);
  }
}
BENCHMARK(BM_x_to_state);

static void BM_state_to_x(benchmark::State& state)
{
  jdrones::data::State s = jdrones::data::State::Random();

  for (auto _ : state)
  {
    jdrones::data::state_to_x(s);
  }
}
BENCHMARK(BM_state_to_x);

BENCHMARK_MAIN();