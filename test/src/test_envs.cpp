/*
 * Copyright (c) 2024.  Jan-Hendrik Ewers
 * SPDX-License-Identifier: GPL-3.0-only
 */

#include <jdrones/dynamics.h>

#include <catch2/catch_all.hpp>

#include "jdrones/envs.h"

using namespace jdrones::envs;

TEMPLATE_TEST_CASE(
    "Env can can be instantiated",
    "[env]",
    LQRDroneEnv,
    FifthOrderPolyPositionDroneEnv,
    OptimalFifthOrderPolyPositionDroneEnv)
{
  float dt = GENERATE(0.01, 0.02, 0.05);
  DYNAMIC_SECTION("dt=" << dt)
  {
    THEN("Env instantiates")
    {
      TestType env(dt);
      REQUIRE((env.get_dt()) == dt);
    }
  }
}

TEST_CASE("LQR drone env can hover", "[env,lqr]")
{
  double dt = GENERATE(0.01, 0.005, 0.001);
  double T = GENERATE(0.1, 10, 100);
  LQRDroneEnv env(dt);
  env.reset();
  State setpoint = State::Zero(), observation;
  DYNAMIC_SECTION("At origin for " << T << " seconds with dt=" << dt)
  {
    for (int i = 0; i < T / dt; ++i)
    {
      observation = std::get<0>(env.step(setpoint));
    }
    double observation_sum = observation.sum();
    bool is_nan = std::isnan(observation_sum);
    REQUIRE(!is_nan);
  }
}

TEMPLATE_TEST_CASE(
    "Polynomial drone envs can go to a position",
    "[env,polynomial]",
    FifthOrderPolyPositionDroneEnv,
    OptimalFifthOrderPolyPositionDroneEnv)
{
  double dt = GENERATE(0.01, 0.005, 0.001);
  double p = GENERATE(1, 10, -10);
  VEC3 setpoint = VEC3::Constant(p);
  DYNAMIC_SECTION("From origin to (" << p << "," << p << "," << p << ") with dt=" << dt)
  {
    TestType env(dt);
    env.reset();

    std::tuple<States, double, bool, bool> observation = env.step(setpoint);
    bool term = std::get<2>(observation);
    bool trunc = std::get<3>(observation);

    States observations = std::get<0>(observation);
    bool is_nan = false;
    for (State s : observations)
    {
      is_nan = std::isnan(s.sum());

      if (is_nan)
      {
        break;
      }
    }

    REQUIRE(!is_nan);
    REQUIRE(!trunc);
    REQUIRE(term);
  }
}
