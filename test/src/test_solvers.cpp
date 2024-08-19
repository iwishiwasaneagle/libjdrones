/*
 * Copyright (c) 2024.  Jan-Hendrik Ewers
 * SPDX-License-Identifier: GPL-3.0-only
 */
#include <jdrones/solvers.h>

#include <catch2/catch_all.hpp>
#include <cmath>

#include "utils.h"

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
      jdrones::types::VEC2 roots = quadratic_roots(1, -2, 0.5);
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

template<int xdim, int udim>
void test_case(
    const Eigen::Matrix<double, xdim, xdim> &A,
    const Eigen::Matrix<double, xdim, udim> &B,
    const Eigen::Matrix<double, xdim, xdim> &Q,
    const Eigen::Matrix<double, udim, udim> &R,
    const Eigen::Matrix<double, xdim, xdim> &exp_P)
{
  Eigen::Matrix<double, xdim, xdim> P;
  P.setZero();
  bool success = solveRiccatiArimotoPotter<xdim, udim>(A, B, Q, R, P);
  REQUIRE(success);
  Eigen::Vector<double, xdim * xdim> P_vec;
  P_vec << P.reshaped();
  Eigen::Vector<double, xdim * xdim> exp_P_vec;
  exp_P_vec << exp_P.reshaped();
  REQUIRE_THAT(
      P_vec,
      Catch::Matchers::RangeEquals(
          exp_P_vec, std::bind(almost_equal_tol, std::placeholders::_1, std::placeholders::_2, 1e-5)));
}

/**
 * Test cases calculated using scipy.linalg.solve_continuous_are
 **/
TEST_CASE("Solve C-ARE", "[solvers,ricatti]")
{
  SECTION("Case 1")
  {
    const uint xdim = 2;
    const uint udim = 1;
    Eigen::Matrix<double, xdim, xdim> A = Eigen::Matrix<double, xdim, xdim>::Zero();
    A.diagonal() << 0.1, 2;
    Eigen::Matrix<double, xdim, xdim> Q = Eigen::Matrix<double, xdim, xdim>::Zero();
    Q.diagonal() << 0, 1;
    Eigen::Matrix<double, xdim, xdim> exp_P{ { 0.3023387, -0.5482406 }, { -0.5482406, 5.23021047 } };
    Eigen::Matrix<double, xdim, udim> B = Eigen::Matrix<double, xdim, udim>::Ones();
    Eigen::Matrix<double, udim, udim> R{ 1. };
    test_case<xdim, udim>(A, B, Q, R, exp_P);
  }
  SECTION("Case 2")
  {
    const uint xdim = 3;
    const uint udim = 1;
    Eigen::Matrix<double, xdim, xdim> A = Eigen::Matrix<double, xdim, xdim>::Zero();
    A(0, 0) = 1;
    A(2, 1) = 2;
    A(2, 2) = 1;
    A(0, 2) = -1;
    Eigen::Matrix<double, xdim, xdim> Q = Eigen::Matrix<double, xdim, xdim>::Zero();
    Q.diagonal() << 0, 1, 1;
    Eigen::Matrix<double, xdim, xdim> exp_P{ { 16.70482006, -22.48492734, -21.48492734 },
                                             { -22.48492734, 33.80907392, 31.80907392 },
                                             { -21.48492734, 31.80907392, 31.30907392 } };
    Eigen::Matrix<double, xdim, udim> B{ 1, 1, 0};
    Eigen::Matrix<double, udim, udim> R{ 1. };

    test_case<xdim, udim>(A, B, Q, R, exp_P);
  }
  SECTION("Case 3")
  {
    const uint xdim = 3;
    const uint udim = 2;
    Eigen::Matrix<double, xdim, xdim> A = Eigen::Matrix<double, xdim, xdim>::Zero();
    A.diagonal() << 0.1, 2, -1;
    Eigen::Matrix<double, xdim, xdim> Q = Eigen::Matrix<double, xdim, xdim>::Zero();
    Q.diagonal() << 10, 1, 0.001;
    Eigen::Matrix<double, xdim, xdim> exp_P{ { 1.00105009e-01, -5.18322537e-05, -4.95425321e-05 },
                                             { -5.18322537e-05, 3.42336206e-02, 5.15161514e-04 },
                                             { -4.95425321e-05, 5.15161514e-04, 4.99883187e-04 } };
    Eigen::Matrix<double, xdim, udim> B{ { 1, 0 }, { 0, 1 }, { 0.1, -1 } };
    Eigen::Matrix<double, udim, udim> R{ { 0.001, 0 }, { 0, 0.001 } };
    test_case<xdim, udim>(A, B, Q, R, exp_P);
  }
}
