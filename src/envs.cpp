/*
 * Copyright (c) 2024.  Jan-Hendrik Ewers
 * SPDX-License-Identifier: GPL-3.0-only
 */
#include "jdrones/envs.h"

#include "jdrones/constants.h"

namespace jdrones::envs
{

  data::State LQRDroneEnv::reset(data::State state)
  {
    this->env.reset(state);
    this->controller.reset();

    return this->env.get_state();
  }

  data::State LQRDroneEnv::reset()
  {
    this->env.reset();
    this->controller.reset();

    return this->env.get_state();
  }
  data::State LQRDroneEnv::step(data::State action)
  {
    VEC4 lqr_action = this->controller(this->env.get_state(), action);

    VEC4 delinearised_action{ 0, 0, 0, this->env.get_mass() * constants::g };
    delinearised_action = this->env.rpm2rpyT(delinearised_action + lqr_action);
    delinearised_action = delinearised_action.cwiseMax(0).cwiseSqrt();

    data::State obs = this->env.step(delinearised_action);

    return obs;
  }
}  // namespace jdrones::envs
