/*
 * Copyright (c) 2024.  Jan-Hendrik Ewers
 * SPDX-License-Identifier: GPL-3.0-only
 */
#include <jdrones/solvers.h>

#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <cmath>

using namespace jdrones::solvers;

double func(double x)
{
  return std::pow(x, 3) - x - 2;
}

TEST_CASE("Find the roots of a function using the bisection method", "[solver,bisection]")
{
  double a = GENERATE(0.1, 1);
  double b = GENERATE(1.6, 100);
  DYNAMIC_SECTION("a=" << a << ", b=" << b)
  {
    SECTION("A root will be found")
    {
      double root = bisection(func, 1, 2, 1e-12);
      REQUIRE_THAT(root, Catch::Matchers::WithinRel(1.5213797068, 1e-10));
    }
  }
}
TEST_CASE("Find the roots of a function using the bisection method with right expansion", "[solver,bisection]")
{
  double a = 0.0;
  double b = GENERATE(0.1, 1.5);
  DYNAMIC_SECTION("a=" << a << ", b=" << b)
  {
    SECTION("A root will be found")
    {
      double root = bisection_with_right_expansion(func, 1, 2, 1e-12);
      REQUIRE_THAT(root, Catch::Matchers::WithinRel(1.5213797068, 1e-10));
    }
  }
}

TEST_CASE("Calculate roots of the quadratic equation", "[solver,quadratic]")
{
  SECTION("Roots are real")
  {
    SECTION("x^2-2x+0.5")
    {
     jdrones::types::VEC2 roots =
        quadratic_roots(1, -2, 0.5);
      REQUIRE_THAT(roots(0), Catch::Matchers::WithinRel(1 + 1 / sqrt(2)));
      REQUIRE_THAT(roots(1), Catch::Matchers::WithinRel(1 - 1 / sqrt(2)));
    }
    SECTION("x^2-2x+1")
    {
      jdrones::types::VEC2 roots = quadratic_roots(1, -2, 1);
      REQUIRE_THAT(roots(0), Catch::Matchers::WithinRel(1.));
      REQUIRE_THAT(roots(1), Catch::Matchers::WithinRel(1.));
    }
  }
  SECTION("Roots are imaginary")
  {
    SECTION("x^2-2x+2")
    {
      jdrones::types::VEC2 roots = quadratic_roots(1, -2, 1.1);
      REQUIRE(roots.isZero());
    }
  }
}
