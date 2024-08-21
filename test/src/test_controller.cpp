/*
 * Copyright (c) 2024.  Jan-Hendrik Ewers
 * SPDX-License-Identifier: GPL-3.0-only
 */

#include <jdrones/controllers.h>

#include <catch2/catch_all.hpp>

TEST_CASE("LQR controller with known results", "[lqr,controller]")
{
  SECTION("With given K")
  {
    jdrones::controllers::LQRController<2, 1> controller({ 1, 1 });
    Eigen::Matrix<double, 1, 1> u;
    SECTION("0")
    {
      u = controller({ 0, 0 }, { 0, 0 });
      REQUIRE_THAT(u(0), Catch::Matchers::WithinRel(0.0));
      REQUIRE(controller.get_e().isZero());
    }
    SECTION("1")
    {
      u = controller({ 0, 0 }, { 0, 1 });
      REQUIRE_THAT(u(0), Catch::Matchers::WithinRel(1.0));
      REQUIRE(controller.get_e().cwiseAbs().sum() == 1);
    }
    SECTION("20")
    {
      controller({ 0, 0 }, { -10, 10 });
      REQUIRE(controller.get_e().cwiseAbs().sum() == 20);
    }
  }
  SECTION("Solve for K")
  {
    Eigen::Matrix<double, 2, 2> A, Q;
    A << 1, 0, 0, 1;
    Q << 1, 0, 0, 1;
    Eigen::Matrix<double, 2, 1> B;
    B << 1, 0;
    Eigen::Matrix<double, 1, 1> R;
    R << 1.0;
    jdrones::controllers::LQRController<2, 1> controller(A, B, Q, R);
  }
}
