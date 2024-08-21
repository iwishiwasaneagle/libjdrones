/*
 * Copyright (c) 2024.  Jan-Hendrik Ewers
 * SPDX-License-Identifier: GPL-3.0-only
 */
#include "jdrones/envs/envs.h"

namespace jdrones::envs
{
  std::tuple<State, std::map<std::string, Eigen::VectorXd>> LQRDroneEnv::reset(data::State state)
  {
    this->controller.reset();
    return this->env.reset(state);
  }

  std::tuple<State, std::map<std::string, Eigen::VectorXd>> LQRDroneEnv::reset()
  {
    return this->reset(State::Zero());
  }
  std::tuple<State, double, bool, bool, std::map<std::string, Eigen::VectorXd>> LQRDroneEnv::step(data::State action)
  {
    VEC4 lqr_action = this->controller(state_to_x(this->env.get_state()), state_to_x(action));

    VEC4 delinearised_action{ 0, 0, 0, this->env.get_mass() * constants::g };
    delinearised_action += lqr_action;
    delinearised_action = this->env.rpyT2rpm(delinearised_action);
    delinearised_action = delinearised_action.cwiseMax(0).cwiseSqrt();

    std::tuple<State, double, bool, bool, std::map<std::string, Eigen::VectorXd>> obs =
        this->env.step(delinearised_action);

    return { std::get<0>(obs), 0.0, false, false, {} };
  }

  polynomial::FifthOrderPolynomial FifthOrderPolyPositionDroneEnv::calc_traj(
      VEC3 pos,
      VEC3 vel,
      VEC3 tgt_pos,
      VEC3 tgt_vel,
      std::map<std::string, double> params)
  {
    double max_vel = params["max_vel"];
    double dist = sqrt((tgt_pos - tgt_vel).array().pow(2).sum());
    double T = dist / max_vel;
    polynomial::FifthOrderPolynomial traj(pos, vel, VEC3::Zero(), tgt_pos, tgt_vel, VEC3::Zero(), T);
    traj.solve();
    return traj;
  }
  std::map<std::string, double> FifthOrderPolyPositionDroneEnv::get_traj_params()
  {
    std::map<std::string, double> params;
    params["max_vel"] = this->max_vel;
    return params;
  };
  polynomial::OptimalFifthOrderPolynomial OptimalFifthOrderPolyPositionDroneEnv::calc_traj(
      VEC3 pos,
      VEC3 vel,
      VEC3 tgt_pos,
      VEC3 tgt_vel,
      std::map<std::string, double> params)
  {
    double max_acc = params["max_acc"];
    polynomial::OptimalFifthOrderPolynomial traj(pos, vel, VEC3::Zero(), tgt_pos, tgt_vel, VEC3::Zero(), max_acc);
    traj.solve();
    return traj;
  }
  std::map<std::string, double> OptimalFifthOrderPolyPositionDroneEnv::get_traj_params()
  {
    std::map<std::string, double> params;
    params["max_acc"] = max_acc;
    return params;
  };

}  // namespace jdrones::envs
