/*
 * Copyright (c) 2024.  Jan-Hendrik Ewers
 * SPDX-License-Identifier: GPL-3.0-only
 */

#include <jdrones/dynamics.h>

#include <catch2/catch_all.hpp>

using namespace jdrones::dynamics;

class TestLinearDynamicModelDroneEnv : public LinearDynamicModelDroneEnv
{
 public:
  using LinearDynamicModelDroneEnv::LinearDynamicModelDroneEnv;

 protected:
  State calc_dstate(Eigen::Vector4d rpm) override
  {
    return LinearDynamicModelDroneEnv::calc_dstate(rpm.array().pow(2));
  }
};

TEMPLATE_TEST_CASE("Envs can be instantiated", "[env]", TestLinearDynamicModelDroneEnv, NonlinearDynamicModelDroneEnv)
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
TEMPLATE_TEST_CASE(
    "Envs behave as expected based on input and single step",
    "[env]",
    TestLinearDynamicModelDroneEnv,
    NonlinearDynamicModelDroneEnv)
{
  float dt = 0.01;
  State state = State::Zero();
  State obs;
  TestType env(dt);
  double hover_mag = sqrt(env.get_mass() * jdrones::constants::g / (4 * env.get_k_t()));
  SECTION("Run for multiple time steps")
  {
    double T = GENERATE(0.1, 10);
    DYNAMIC_SECTION("At origin for " << T << " seconds with dt=" << dt)
    {
      TestType env(dt);
      env.reset();
      VEC4 u = VEC4::Constant(hover_mag);
      State observation;
      for (int i = 0; i < T / dt; ++i)
      {
        observation = env.step(u);
      }
      double observation_sum = observation.sum();
      bool is_nan = isnanf(observation_sum);
      REQUIRE(!is_nan);
    }
  }
  SECTION("Constant input")
  {
    double yaw = GENERATE(0, 0.1);
    state.set_ang_vel({ 0, 0, yaw });
    env.reset(state);
    SECTION("Zero or low input")
    {
      double mag = GENERATE(0, 0.1);
      Eigen::Vector4d u = Eigen::Vector4d::Constant(mag);
      obs = env.step(u);
      SECTION("Velocity is downward")
      {
        Eigen::Vector3d vel = obs.get_vel();
        REQUIRE(vel.block<2, 1>(0, 0).isZero());
        REQUIRE(vel(2) < 0);
      }
      SECTION("Angular velocity is zero")
      {
        Eigen::Vector3d ang_vel = obs.get_ang_vel();
        REQUIRE(ang_vel.isZero());
      }
    }
    SECTION("Large input")
    {
      double mag = GENERATE(100, 1e4);
      Eigen::Vector4d u = Eigen::Vector4d::Constant(mag);
      obs = env.step(u);
      SECTION("Velocity is upward")
      {
        Eigen::Vector3d vel = obs.get_vel();
        REQUIRE(vel.block<2, 1>(0, 0).isZero());
        REQUIRE(vel(2) > 0);
      }
      SECTION("Angular velocity is zero")
      {
        Eigen::Vector3d ang_vel = obs.get_ang_vel();
        REQUIRE(ang_vel.isZero());
      }
    }
  }
  SECTION("Angle input expecations")
  {
    env.reset();
    SECTION("Roll input")
    {
      Eigen::Matrix<double, 7, 4> vec_omega{ { 1, 1, 1, 1 },     { 1, 0.4, 1, 0.5 }, { 1, 0.5, 1, 0.4 }, { 1, 0.9, 1, 1.1 },
                                             { 1, 1.1, 1, 0.9 }, { 1, 2, 1, 0 },     { 1, 0, 1, 2 } };
      Eigen::Matrix<double, 7, 3> exp{ { 0, 0, 0 },   { 1, 0, 1 },   { -1, 0, 1 }, { 1, 0, -1 },
                                       { -1, 0, -1 }, { -1, 0, -1 }, { 1, 0, -1 } };
      int i = GENERATE(range(0, 6));
      DYNAMIC_SECTION("Sign matches expected for test case " << i)
      {
        obs = env.step(hover_mag * vec_omega.row(i));
        REQUIRE_THAT(obs.get_ang_vel().cwiseSign(), Catch::Matchers::RangeEquals(exp.row(i)));
      }
    }
    SECTION("Pitch input")
    {
      Eigen::Matrix<double, 7, 4> vec_omega{
        { 1, 1, 1, 1 },     { 0.5, 1, 0.4, 1 }, { 0.4, 1, 0.5, 1 }, { 1.1, 1, 0.9, 1 },
        { 0.9, 1, 1.1, 1 }, { 2, 1, 0, 1 },     { 0, 1, 2, 1 },

      };
      Eigen::Matrix<double, 7, 3> exp{
        { 0, 0, 0 }, { 0, -1, -1 }, { 0, 1, -1 }, { 0, -1, 1 }, { 0, 1, 1 }, { 0, -1, 1 }, { 0, 1, 1 },
      };
      int i = GENERATE(range(0, 6));
      DYNAMIC_SECTION("Sign matches expected for test case " << i)
      {
        obs = env.step(hover_mag * vec_omega.row(i));
        REQUIRE_THAT(obs.get_ang_vel().cwiseSign(), Catch::Matchers::RangeEquals(exp.row(i)));
      }
    }
    SECTION("Yaw input")
    {
      Eigen::Matrix<double, 5, 4> vec_omega{
        { 1, 1, 1, 1 }, { 1, 1.1, 1, 1.1 }, { 1.1, 1, 1.1, 1 }, { 1, 2, 1, 2 }, { 2, 1, 2, 1 },

      };
      Eigen::Matrix<double, 5, 3> exp{
        { 0, 0, 0 }, { 0, 0, -1 }, { 0, 0, 1 }, { 0, 0, -1 }, { 0, 0, 1 },
      };
      int i = GENERATE(range(0, 4));
      DYNAMIC_SECTION("Sign matches expected for test case " << i)
      {
        obs = env.step(hover_mag * vec_omega.row(i));
        REQUIRE_THAT(obs.get_ang_vel().cwiseSign(), Catch::Matchers::RangeEquals(exp.row(i)));
      }
    }
  }
  SECTION("Velocity input expectations")
  {
    SECTION("Velocity from rotation")
    {
      Eigen::Matrix<double, 7, 3> rpy{ { 0, 0, 0 },    { 0, 0, 0.1 }, { 0, 0, -0.1 }, { 0.1, 0, 0 },
                                       { -0.1, 0, 0 }, { 0, 0.1, 0 }, { 0, -0.1, 0 } };

      Eigen::Matrix<double, 7, 3> exp{ { 0, 0, -1 }, { 0, 0, -1 }, { 0, 0, -1 }, { 0, -1, -1 },
                                       { 0, 1, -1 }, { 1, 0, -1 }, { -1, 0, -1 } };
      int i = GENERATE(range(0, 6));
      DYNAMIC_SECTION("Sign matches expected for test case " << i)
      {
        state.set_rpy(rpy.row(i));
        state.set_quat(jdrones::euler_to_quat(state.get_rpy()));
        env.reset(state);

        obs = env.step(VEC4::Constant(0.99 * hover_mag));
        REQUIRE_THAT(obs.get_vel().cwiseSign(), Catch::Matchers::RangeEquals(exp.row(i)));
      }
    }
    SECTION("Position from velocity")
    {
      Eigen::Matrix<double, 10, 3> vel{ { 1, 1, 1 },   { 1, 1, 0 }, { 1, 1, -1 }, { 1, 0, 0 },   { 1, 0, -1 },
                                        { 1, -1, -1 }, { 0, 0, 0 }, { 0, 0, -1 }, { 0, -1, -1 }, { -1, -1, -1 } };
      int i = GENERATE(range(0, 9));
      DYNAMIC_SECTION("Sign matches expected for test case " << i)
      {
        state.set_vel(vel.row(i));
        env.reset(state);

        obs = env.step(VEC4::Zero());
        REQUIRE_THAT(obs.get_pos().cwiseSign(), Catch::Matchers::RangeEquals(vel.row(i)));
      }
    }
  }

  SECTION("Angular velocity input expectations")
  {
    SECTION("RPY from angular velocity")
    {
      Eigen::Matrix<double, 10, 3> ang_vel{ { 1, 1, 1 },   { 1, 1, 0 }, { 1, 1, -1 }, { 1, 0, 0 },   { 1, 0, -1 },
                                            { 1, -1, -1 }, { 0, 0, 0 }, { 0, 0, -1 }, { 0, -1, -1 }, { -1, -1, -1 } };

      int i = GENERATE(range(0, 9));
      DYNAMIC_SECTION("Sign matches expected for test case " << i)
      {
        state.set_ang_vel(ang_vel.row(i));
        env.reset(state);

        obs = env.step(VEC4::Zero());
        REQUIRE_THAT(obs.get_rpy().cwiseSign(), Catch::Matchers::RangeEquals(ang_vel.row(i)));
      }
    }
    SECTION("Input to angular velocity")
    {
      Eigen::Matrix<double, 2, 4> u{
        { 1, 0.9, 1, 0.9 },
        { 0.9, 1, 0.9, 1 },

      };
      Eigen::Matrix<double, 2, 3> exp{
        { 0, 0, 1 },
        { 0, 0, -1 },
      };

      int i = GENERATE(range(0, 2));
      DYNAMIC_SECTION("Sign matches expected for test case " << i)
      {
        obs = env.step(100 * u.row(i));
        REQUIRE_THAT(obs.get_ang_vel().cwiseSign(), Catch::Matchers::RangeEquals(exp.row(i)));
      }
    }
  }
}