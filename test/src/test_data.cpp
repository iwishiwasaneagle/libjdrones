/*
 * Copyright (c) 2024.  Jan-Hendrik Ewers
 * SPDX-License-Identifier: GPL-3.0-only
 */

#include <jdrones/data.h>

#include <catch2/catch_all.hpp>

using namespace jdrones::data;

TEMPLATE_TEST_CASE("Data type can be instantiated", "[data]", State, X)
{
  SECTION("With zeros")
  {
    TestType t = TestType::Zero();
    THEN("Is all zeros")
    {
      REQUIRE(t.cwiseAbs().sum() == 0);
    }
  }
  SECTION("With random")
  {
    TestType t = TestType::Random();
    THEN("Is not all zeros")
    {
      REQUIRE(t.cwiseAbs().sum() > 0);
    }
  }
}
