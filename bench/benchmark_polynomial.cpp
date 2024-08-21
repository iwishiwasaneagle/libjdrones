/*
 * Copyright (c) 2024.  Jan-Hendrik Ewers
 * SPDX-License-Identifier: GPL-3.0-only
 */

#include <benchmark/benchmark.h>

#include "jdrones/data.h"
#include "jdrones/polynomial.h"

using namespace jdrones::data;

template<class Poly>
Poly make_poly()
{
  VEC3 zero = VEC3::Zero();
  VEC3 ten = VEC3::Constant(10);
  Poly poly(zero, zero, zero, ten, ten, ten);
  poly.solve();
  return poly;
}

void BM_make_fifth_order(benchmark::State& state)
{
  for (auto _ : state)
  {
    make_poly<jdrones::polynomial::FifthOrderPolynomial>();
  }
}
BENCHMARK(BM_make_fifth_order);

void BM_make_opt_fifth_order(benchmark::State& state)
{
  for (auto _ : state)
  {
    make_poly<jdrones::polynomial::OptimalFifthOrderPolynomial>();
  }
}
BENCHMARK(BM_make_fifth_order);

void BM_position(benchmark::State& state)
{
  auto poly = make_poly<jdrones::polynomial::FifthOrderPolynomial>();
  for (auto _ : state)
  {
    poly.position(0.0);
  }
}
BENCHMARK(BM_position);

void BM_velocity(benchmark::State& state)
{
  auto poly = make_poly<jdrones::polynomial::FifthOrderPolynomial>();
  for (auto _ : state)
  {
    poly.velocity(0.0);
  }
}
BENCHMARK(BM_velocity);

void BM_acceleration(benchmark::State& state)
{
  auto poly = make_poly<jdrones::polynomial::FifthOrderPolynomial>();
  for (auto _ : state)
  {
    poly.acceleration(0.0);
  }
}
BENCHMARK(BM_acceleration);
void BM_jerk(benchmark::State& state)
{
  auto poly = make_poly<jdrones::polynomial::FifthOrderPolynomial>();
  for (auto _ : state)
  {
    poly.jerk(0.0);
  }
}
BENCHMARK(BM_jerk);

BENCHMARK_MAIN();