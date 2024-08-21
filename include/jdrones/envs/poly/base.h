/*
 * Copyright (c) 2024.  Jan-Hendrik Ewers
 * SPDX-License-Identifier: GPL-3.0-only
 */

#ifndef ENVS_POLY_BASE_H
#define ENVS_POLY_BASE_H
#include <map>

#include <eigen3/Eigen/Core>
#include "jdrones/data.h"
#include "jdrones/polynomial.h"

namespace jdrones::envs
{
  using namespace jdrones::data;
  template<class Polynomial>
  class BasePolynomialPositionDroneEnv : public gymnasium::Env<States, State, VEC3>
  {
   protected:
    double dt;
    LQRDroneEnv env;

   public:
    [[nodiscard]] LQRDroneEnv get_env() const
    {
      return env;
    }
    [[nodiscard]] double get_dt() const
    {
      return dt;
    }
    virtual Polynomial calc_traj(VEC3 pos, VEC3 vel, VEC3 tgt_pos, VEC3 tgt_vel, std::map<std::string, double> params) = 0;
    virtual std::map<std::string, double> get_traj_params() = 0;

    BasePolynomialPositionDroneEnv(double dt, data::State state) : dt(dt), env(dt, state)
    {
    }
    BasePolynomialPositionDroneEnv(double dt) : BasePolynomialPositionDroneEnv(dt, State::Zero())
    {
    }

    std::tuple<States, std::map<std::string, Eigen::VectorXd>> reset() override
    {
      return this->reset(State::Zero());
    }
    std::tuple<States, std::map<std::string, Eigen::VectorXd>> reset(State state) override
    {
      std::tuple<State, std::map<std::string, Eigen::VectorXd>> result = this->env.reset(state);
      return { { std::get<0>(result) }, std::get<1>(result) };
    }
    std::tuple<States, double, bool, bool, std::map<std::string, Eigen::VectorXd>> step(VEC3 action) override
    {
      VEC3 vel = VEC3::Zero();
      return this->step(std::pair<VEC3, VEC3>{ action, vel });
    }
    std::tuple<States, double, bool, bool, std::map<std::string, Eigen::VectorXd>> step(std::pair<VEC3, VEC3> action)
    {
      VEC3 tgt_pos = action.first;
      VEC3 tgt_vel = action.second;

      bool term = false, trunc = false;

      State cur_state = this->env.get_env().get_state();
      VEC3 cur_pos = cur_state.get_pos(), cur_vel = cur_state.get_vel();

      if ((cur_pos - tgt_pos).cwiseAbs().isZero(1e-9))
      {
        return { { cur_state }, 0.0, term, true, {} };
      }
      Polynomial traj = calc_traj(cur_pos, cur_vel, tgt_pos, tgt_vel, get_traj_params());
      std::vector<State> observations{ cur_state };

      State u = State::Zero();
      int counter = -1;
      double t, dist;

      do
      {
        t = counter++ * this->dt;
        if (t >= traj.get_T())
        {
          u.set_pos(tgt_pos);
          u.set_vel(VEC3::Zero());
        }
        else
        {
          u.set_pos(traj.position(t));
          u.set_vel(traj.velocity(t));
        }

        std::tuple<State, double, bool, bool, std::map<std::string, Eigen::VectorXd>> env_return = this->env.step(u);

        cur_state = std::get<0>(env_return);
        observations.push_back(cur_state);

        dist = (cur_state.get_pos() - tgt_pos).norm();
        if (dist < 0.5)
        {
          term = true;
        }
        else if (std::isnan(dist))
        {
          trunc = true;
        }
      } while (!(term || trunc));
      return { observations, 0.0, term, trunc, {} };
    }
  };
}  // namespace jdrones::envs
#endif  // ENVS_POLY_BASE_H
