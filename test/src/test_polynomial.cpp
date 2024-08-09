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

TEST_CASE("Increased T reduces the 2nd derivative (acceleration)", "[polynomial]")
{
  BMATRIX b;
  b.setZero();
  b.row(0).setConstant(10.0);
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
  int i = GENERATE(range(0, 3));
  DYNAMIC_SECTION("Along dim " << i)
  {
    float T = GENERATE(1, 100);
    float px = GENERATE(-1, -100);
    float vx = GENERATE(0, 100);
    float ax = GENERATE(0, 1);
    float ptx = GENERATE(10, 100);
    float vtx = GENERATE(0, 100);
    float atx = GENERATE(0, 1);

    Eigen::Vector3d p, v, a, tp, tv, ta, act;
    p.setZero();
    v.setZero();
    a.setZero();
    tp.setZero();
    tv.setZero();
    ta.setZero();
    p(i) = px;
    v(i) = vx;
    a(i) = ax;
    tp(i) = ptx;
    tv(i) = vtx;
    ta(i) = atx;
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
      SECTION("Inferenced acceleration at along polynomial is below threshold")
      {
        if (!std::is_same_v<TestType, OptimalFifthOrderPolynomial>)
        {
          SKIP("Test case only applicable for OptimalFifthOrderPolynomial");
        }
        else
        {
          VEC3 acceleration, diff, zero_clipped_diff;
          Eigen::Vector<double, 100> times = Eigen::VectorXd::LinSpaced(100, 0.0, polynomial.get_T());
          double max_acc = 1.0;
          for (int i = 0; i < 100; i++)
          {
            acceleration = polynomial.acceleration(times(i));
            diff = acceleration.cwiseAbs().array() - max_acc;
            zero_clipped_diff = diff.cwiseMax(0);
            REQUIRE_THAT(zero_clipped_diff, Catch::Matchers::RangeEquals(VEC3::Zero(), almost_equal));
          }
        }
      }
    }
  }
}

TEST_CASE("Coefficients of the derivatives are being correctly calculated", "[polynomial]")
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
