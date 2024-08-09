/*
 * Copyright (c) 2024.  Jan-Hendrik Ewers
 * SPDX-License-Identifier: GPL-3.0-only
 */

#include <jdrones/envs.h>

#include <catch2/catch_all.hpp>

using namespace jdrones::envs;

TEMPLATE_TEST_CASE("Envs can be instantiated", "[env]", LinearDynamicModelDroneEnv, NonlinearDynamicModelDroneEnv)
{
  float dt = GENERATE(0.01, 0.02, 0.05);
  DYNAMIC_SECTION("dt=" << dt)
  {
    THEN("Env instantiates")
    {
      TestType env(dt);
      REQUIRE((env.dt) == dt);
    }
  }
}
TEMPLATE_TEST_CASE("Envs behave as expected", "[env]", LinearDynamicModelDroneEnv, NonlinearDynamicModelDroneEnv)
{
  float dt = 0.01;
  SECTION("Test velocity after step based on input")
  {
    TestType env(dt);
    env.reset();
    THEN("Velocity is downward with zero input")
    {
      Eigen::Vector4d u{0,0,0,0};
      State state = env.step(u);
      Eigen::Vector3d vel = state.get_vel();
      REQUIRE(vel(2) < 0);
    }
    THEN("Velocity is upward with large positive constant input")
    {
      Eigen::Vector4d u{ 1000, 1000, 1000, 1000 };
      State state = env.step(u);
      Eigen::Vector3d vel = state.get_vel();
      REQUIRE(vel(2) > 0);
    }
  }
}
