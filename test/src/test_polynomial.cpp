/*
 * Copyright (c) 2024.  Jan-Hendrik Ewers
 * SPDX-License-Identifier: GPL-3.0-only
 */
#include <jdrones/polynomial.h>

#include <catch2/catch_all.hpp>

using namespace jdrones::polynomial;

static bool almost_equal(const double& lhs, const double& rhs)
{
  return std::abs(lhs - rhs) < 1e-9;
}

TEMPLATE_TEST_CASE("Different polynomial classes","[trajectory,polynomial]", FifthOrderPolynomial, OptimalFifthOrderPolynomial){
    float T = GENERATE(1, 10, 100);
    float px = GENERATE(-1, -10, -100);
    float vx = GENERATE(0, 1, 10, 100);
    float ax = GENERATE(0, 1, 10, 100);
    float ptx = GENERATE(1, 10, 100);
    float vtx = GENERATE(0, 1, 10, 100);
    float atx = GENERATE(0, 1, 10, 100);
    Eigen::Vector3d p, v, a, tp, tv, ta, act_p, act_v, act_a;
    p << px, 0, 0;
    v << vx, 0, 0;
    a << ax, 0, 0;
    tp << ptx, 0, 0;
    tv << vtx, 0, 0;
    ta << atx, 0, 0;
    DYNAMIC_SECTION(
        "With changing p=" << p << ",v=" << v << ",a=" << a << "p=" << tp << ",v=" << tv << ",a=" << ta << " and T=" << T)
    {
      TestType polynomial(p, v, a, tp, tv, ta, T);
      THEN("Coeffs is not zero")
      {
        REQUIRE(polynomial.get_coeffs().cwiseAbs().sum() > 0);
      }
      THEN("Inferenced position at t=0 is the same as bound")
      {
        act_p = polynomial.position(0);
        REQUIRE_THAT(act_p, Catch::Matchers::RangeEquals(p, almost_equal));
      }
      THEN("Inferenced position at t=T is the same as bound")
      {
        act_p = polynomial.position(T);
        REQUIRE_THAT(act_p, Catch::Matchers::RangeEquals(tp, almost_equal));
      }
      THEN("Inferenced velocity at t=0 is the same as bound")
      {
        act_v = polynomial.velocity(0);
        REQUIRE_THAT(act_v, Catch::Matchers::RangeEquals(v, almost_equal));
      }
      THEN("Inferenced velocity at t=T is the same as bound")
      {
        act_v = polynomial.velocity(T);
        REQUIRE_THAT(act_v, Catch::Matchers::RangeEquals(tv, almost_equal));
      }
      THEN("Inferenced acceleration at t=0 is the same as bound")
      {
        act_a = polynomial.acceleration(0);
        REQUIRE_THAT(act_a, Catch::Matchers::RangeEquals(a, almost_equal));
      }
      THEN("Inferenced acceleration at t=T is the same as bound")
      {
        act_a = polynomial.acceleration(T);
        REQUIRE_THAT(act_a, Catch::Matchers::RangeEquals(ta, almost_equal));
      }
  }
}
