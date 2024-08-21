/*
 * Copyright (c) 2024.  Jan-Hendrik Ewers
 * SPDX-License-Identifier: GPL-3.0-only
 */

#ifndef LQR_H
#define LQR_H
#include <eigen3/Eigen/Core>
#include "jdrones/controllers.h"
#include "jdrones/data.h"
#include "jdrones/dynamics/nonlinear.h"
#include "jdrones/gymnasium.h"

namespace jdrones::envs
{
  using namespace jdrones::data;
  class LQRDroneEnv : public gymnasium::Env<State, State, State>
  {
    controllers::LQRController<12, 4> controller;
    dynamics::NonlinearDynamicModelDroneEnv env;

   public:
    LQRDroneEnv(double dt, State state, Eigen::Matrix<double, 4, 12> K) : env(dt, state), controller(K)
    {
    }
    LQRDroneEnv(double dt, State state) : LQRDroneEnv(dt, state, Eigen::Matrix<double, 4, 12>::Zero())
    {
      Eigen::Matrix<double, 12, 12> Q = Eigen::Matrix<double, 12, 12>::Zero();
      Q.diagonal() << 0.00013741387768501927, 0.00014918283841067683, 0.0001468558043779094, 7.157737996308742e-05,
          0.00012850431641269944, 1.4566003039918306e-06, 3.28709705868307e-05, 4.0376730414403854e-05,
          0.00016339255544858106, 6.637551646435567e-05, 0.0001076879654213928, 6.371223841699211e-05;

      Eigen::Matrix<double, 4, 4> R = Eigen::Matrix<double, 4, 4>::Zero();
      R.diagonal() << 0.1335922092065498, 0.2499451121859131, 35.41422613197229, 4.854927340822368e-05;
      controller = controllers::LQRController<12, 4>(this->env.get_A(), this->env.get_B(), Q, R);
    }
    LQRDroneEnv(double dt) : LQRDroneEnv(dt, State::Zero())
    {
    }
    std::tuple<State, std::map<std::string, Eigen::VectorXd>> reset(State state) override;
    std::tuple<State, std::map<std::string, Eigen::VectorXd>> reset() override;
    std::tuple<State, double, bool, bool, std::map<std::string, Eigen::VectorXd>> step(State action) override;

    [[nodiscard]] dynamics::NonlinearDynamicModelDroneEnv get_env()
    {
      return this->env;
    }
    [[nodiscard]] double get_dt()
    {
      return this->env.get_dt();
    }
  };
}  // namespace jdrones::envs

#endif  // LQR_H
