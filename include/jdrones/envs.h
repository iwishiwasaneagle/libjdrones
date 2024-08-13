/*
 * Copyright (c) 2024.  Jan-Hendrik Ewers
 * SPDX-License-Identifier: GPL-3.0-only
 */

#ifndef ENVS_H
#define ENVS_H

#include "jdrones/controllers.h"
#include "jdrones/dynamics.h"

namespace jdrones::envs
{
  class LQRDroneEnv
  {
    dynamics::NonlinearDynamicModelDroneEnv env;
    controllers::LQRController controller;

   public:

    LQRDroneEnv(double dt, data::State state, Eigen::Matrix<double, 4, 12> K) : env(dt, state), controller(K)
    {
    }
    LQRDroneEnv(double dt, data::State state) : env(dt, state), controller(Eigen::Matrix<double, 4, 12>::Zero())
    {
    }
    LQRDroneEnv(double dt) : env(dt), controller(Eigen::Matrix<double, 4, 12>::Zero())
    {
    }
    data::State reset(data::State state);
    data::State reset();
    data::State step(data::State action);

    dynamics::NonlinearDynamicModelDroneEnv get_env()
    {
      return env;
    };
    void set_K(Eigen::Matrix<double, 4, 12> K)
    {
      this->controller.set_K(K);
    }
  };
}  // namespace jdrones::envs

#endif  // ENVS_H
