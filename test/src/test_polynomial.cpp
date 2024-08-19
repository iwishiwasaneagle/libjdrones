/*
 * Copyright (c) 2024.  Jan-Hendrik Ewers
 * SPDX-License-Identifier: GPL-3.0-only
 */
#include <jdrones/polynomial.h>

#include <catch2/catch_all.hpp>
#include "utils.h"

using namespace jdrones::polynomial;



TEST_CASE("Increased T reduces the 2nd derivative (acceleration)", "[polynomial]")
{
  BMATRIX b;
  b.setZero();
  b.row(0).setConstant(10.0);
  double T1 = 1, T2 = 10;

  SECTION("Along FifthOrderPolynomial trajectory")
  {
    FifthOrderPolynomial traj1(b, T1);
    FifthOrderPolynomial traj2(b, T2);

    double a1, a2;
    int N = 100;
    for (int i = 0; i <= N; i++)
    {
      a1 = traj1.acceleration(traj1.get_T() * i / N)(0);
      a2 = traj2.acceleration(traj2.get_T() * i / N)(0);
      REQUIRE(a1 + 1e-20 >= a2);
    }
  }
  SECTION("OptimalFifthOrderPolynomial::get_global_max_abs_accelerations_from_time with different times")
  {
    double a1 = OptimalFifthOrderPolynomial::get_global_max_abs_accelerations_from_time(b, T1);
    double a2 = OptimalFifthOrderPolynomial::get_global_max_abs_accelerations_from_time(b, T2);
    REQUIRE(a1 > a2);
  }
}

TEMPLATE_TEST_CASE(
    "Different polynomial classes",
    "[trajectory,polynomial]",
    FifthOrderPolynomial,
    OptimalFifthOrderPolynomial)
{
  float T = GENERATE(1, 5, 100);
  float px = GENERATE(-1, -100);
  float vx = GENERATE(-1, 1);
  float ax = GENERATE(-1, 1);
  float ptx = GENERATE(10, 100);
  float vtx = GENERATE(1, 2);
  float atx = GENERATE(1, 2);

  Eigen::Vector3d p, v, a, tp, tv, ta, act;
  p.setConstant(px);
  v.setConstant(vx);
  a.setConstant(ax);
  tp.setConstant(ptx);
  tv.setConstant(vtx);
  ta.setConstant(atx);
  DYNAMIC_SECTION(
      "With changing p=" << p << ",v=" << v << ",a=" << a << "p=" << tp << ",v=" << tv << ",a=" << ta << " and T=" << T)
  {
    TestType polynomial(p, v, a, tp, tv, ta, T);
    polynomial.solve();
    SECTION("T is reasonable")
    {
      REQUIRE(polynomial.get_T() < 1e4);
    }
    SECTION("b matrix is correct")
    {
      BMATRIX b = polynomial.get_b_matrix();
      REQUIRE_THAT(b.row(0), Catch::Matchers::RangeEquals(p, almost_equal));
      REQUIRE_THAT(b.row(2), Catch::Matchers::RangeEquals(v, almost_equal));
      REQUIRE_THAT(b.row(4), Catch::Matchers::RangeEquals(a, almost_equal));
      REQUIRE_THAT(b.row(1), Catch::Matchers::RangeEquals(tp, almost_equal));
      REQUIRE_THAT(b.row(3), Catch::Matchers::RangeEquals(tv, almost_equal));
      REQUIRE_THAT(b.row(5), Catch::Matchers::RangeEquals(ta, almost_equal));
    }
    SECTION("Coeffs is not zero")
    {
      REQUIRE(polynomial.get_coeffs().cwiseAbs().sum() > 0);
    }
    SECTION("Inferenced position at t=0 is the same as bound")
    {
      act = polynomial.position(0);
      REQUIRE_THAT(act, Catch::Matchers::RangeEquals(p, almost_equal));
    }
    SECTION("Inferenced position at t=T is the same as bound")
    {
      act = polynomial.position(polynomial.get_T());
      REQUIRE_THAT(act, Catch::Matchers::RangeEquals(tp, almost_equal));
    }
    SECTION("Inferenced velocity at t=0 is the same as bound")
    {
      act = polynomial.velocity(0);
      REQUIRE_THAT(act, Catch::Matchers::RangeEquals(v, almost_equal));
    }
    SECTION("Inferenced velocity at t=T is the same as bound")
    {
      act = polynomial.velocity(polynomial.get_T());
      REQUIRE_THAT(act, Catch::Matchers::RangeEquals(tv, almost_equal));
    }
    SECTION("Inferenced acceleration at t=0 is the same as bound")
    {
      act = polynomial.acceleration(0);
      REQUIRE_THAT(act, Catch::Matchers::RangeEquals(a, almost_equal));
    }
    SECTION("Inferenced acceleration at t=T is the same as bound")
    {
      act = polynomial.acceleration(polynomial.get_T());
      REQUIRE_THAT(act, Catch::Matchers::RangeEquals(ta, almost_equal));
    }
  }
}

TEST_CASE("Coefficients of the derivatives are being correctly calculated", "[polynomial,fifthorder]")
{
  COEFFMAT3 coeffs = COEFFMAT3::Zero();
  coeffs.col(0) << 1, 2, 3, 4, 5, 6;

  SECTION("Velocity")
  {
    COEFFMAT3 exp_coeffs = COEFFMAT3::Zero();
    exp_coeffs.col(0) << 5, 8, 9, 8, 5, 0;
    COEFFMAT3 vel_coeffs = FifthOrderPolynomial::calc_velocity_coeffs(coeffs);
    REQUIRE_THAT(exp_coeffs.col(0), Catch::Matchers::RangeEquals(vel_coeffs.col(0)));
  }
  SECTION("Acceleration")
  {
    COEFFMAT3 exp_coeffs = COEFFMAT3::Zero();
    exp_coeffs.col(0) << 20, 24, 18, 8, 0, 0;
    COEFFMAT3 acc_coeffs = FifthOrderPolynomial::calc_acceleration_coeffs(coeffs);
    REQUIRE_THAT(exp_coeffs.col(0), Catch::Matchers::RangeEquals(acc_coeffs.col(0)));
  }
  SECTION("Jerk")
  {
    COEFFMAT3 exp_coeffs = COEFFMAT3::Zero();
    exp_coeffs.col(0) << 60, 48, 18, 0, 0, 0;
    COEFFMAT3 jerk_coeffs = FifthOrderPolynomial::calc_jerk_coeffs(coeffs);
    REQUIRE_THAT(exp_coeffs.col(0), Catch::Matchers::RangeEquals(jerk_coeffs.col(0)));
  }
  SECTION("Snap")
  {
    COEFFMAT3 exp_coeffs = COEFFMAT3::Zero();
    exp_coeffs.col(0) << 120, 48, 0, 0, 0, 0;
    COEFFMAT3 snap_coeffs = FifthOrderPolynomial::calc_snap_coeffs(coeffs);
    REQUIRE_THAT(exp_coeffs.col(0), Catch::Matchers::RangeEquals(snap_coeffs.col(0)));
  }
}

TEST_CASE("Ensure optimality of the generated polynomial", "[polynomial,optimal]")
{
  double max_acc = GENERATE(1, 10);
  DYNAMIC_SECTION("With max_acc=" << max_acc)
  {
    float T = GENERATE(1, 5, 25, 100);
    DYNAMIC_SECTION("With t_max=" << T)
    {
      Eigen::Matrix<double, 9, 6> inputs{
        { 10, 0, 0, 0, 0, 0 },   { 0, 0, 0, 10, 0, 0 },   { 10, 0, 1, 0, 0, 0 },
        { 0, 0, 0, 10, 0, 1 },   { 10, 1, 0, 0, 0, 0 },   { 0, 0, 0, 10, 1, 0 },
        { -10, 0, 0, 10, 0, 0 }, { -10, 1, 0, 10, 0, 1 }, { -10, -1, 0, -100, 0, -1 },
      };
      int i = GENERATE(range(0, 9));
      Eigen::Vector3d p, v, a, tp, tv, ta;
      p.setConstant(inputs(i, 0));
      v.setConstant(inputs(i, 1));
      a.setConstant(inputs(i, 2));
      tp.setConstant(inputs(i, 3));
      tv.setConstant(inputs(i, 4));
      ta.setConstant(inputs(i, 5));
      DYNAMIC_SECTION("Input set " << i)
      {
        OptimalFifthOrderPolynomial polynomial(p, v, a, tp, tv, ta, T, max_acc, 1e-10, 1000);
        polynomial.solve();
        SECTION("t_optim is not unrealistic")
        {
          REQUIRE(polynomial.get_T() < 1e5);
        }

        SECTION("Inferenced acceleration at along polynomial is below threshold")
        {
          VEC3 acceleration;
          const unsigned int N = 100;
          Eigen::VectorXd times = Eigen::VectorXd::LinSpaced(N, 0.0, polynomial.get_T());

          double max_abs_acc, act_max_abs_acc = -INFINITY;
          for (int i = 0; i < N; i++)
          {
            acceleration = polynomial.acceleration(times(i));
            max_abs_acc = acceleration.cwiseAbs().maxCoeff();
            if (max_abs_acc > act_max_abs_acc)
            {
              act_max_abs_acc = max_abs_acc;
            }
          }

          double calculated_max_acc =
              polynomial.get_global_max_abs_accelerations_from_time(polynomial.get_b_matrix(), polynomial.get_T());

          REQUIRE_THAT(act_max_abs_acc, Catch::Matchers::WithinRel(calculated_max_acc, 1e-3));
          REQUIRE_THAT(act_max_abs_acc, Catch::Matchers::WithinRel(max_acc, 1e-3));
          REQUIRE(act_max_abs_acc > 0.1);
        }
      }
    }
  }
}