/*
 * Copyright (c) 2024.  Jan-Hendrik Ewers
 * SPDX-License-Identifier: GPL-3.0-only
 */

#ifndef ENVS_H
#define ENVS_H

#include <map>

#include "jdrones/controllers.h"
#include "jdrones/dynamics.h"
#include "polynomial.h"

namespace jdrones::envs
{
  class LQRDroneEnv : public dynamics::NonlinearDynamicModelDroneEnv
  {
    controllers::LQRController<12, 4> controller;

   public:
    LQRDroneEnv(double dt, data::State state, Eigen::Matrix<double, 4, 12> K)
        : NonlinearDynamicModelDroneEnv(dt, state),
          controller(K)
    {
    }
    LQRDroneEnv(double dt, data::State state) : LQRDroneEnv(dt, state, Eigen::Matrix<double, 4, 12>::Zero())
    {
      Eigen::Matrix<double, 12, 12> Q = Eigen::Matrix<double, 12, 12>::Zero();
      Q.diagonal() << 0.00013741387768501927, 0.00014918283841067683, 0.0001468558043779094, 7.157737996308742e-05,
          0.00012850431641269944, 1.4566003039918306e-06, 3.28709705868307e-05, 4.0376730414403854e-05,
          0.00016339255544858106, 6.637551646435567e-05, 0.0001076879654213928, 6.371223841699211e-05;

      Eigen::Matrix<double, 4, 4> R = Eigen::Matrix<double, 4, 4>::Zero();
      R.diagonal() << 0.1335922092065498, 0.2499451121859131, 35.41422613197229, 4.854927340822368e-05;
      controller = controllers::LQRController<12, 4>(this->A, this->B, Q, R);
    }
    LQRDroneEnv(double dt) : LQRDroneEnv(dt, State::Zero())
    {
    }
    data::State reset(data::State state);
    data::State reset();
    std::tuple<State, double, bool, bool> step(data::State action);

    void set_K(Eigen::Matrix<double, 4, 12> K)
    {
      this->controller.set_K(K);
    }
  };

  using States = std::vector<State>;

  template<class Polynomial>
  class BasePolynomialPositionDroneEnv
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
    BasePolynomialPositionDroneEnv(double dt, data::State state, Eigen::Matrix<double, 4, 12> K) : dt(dt), env(dt, state, K)
    {
    }

    std::vector<State> reset()
    {
      auto reset_return = this->env.reset();
      States states = static_cast<States>(reset_return(0));
      return states;
    }
    States reset(State state)
    {
      auto reset_return = this->env.reset(state);
      States states = static_cast<States>(reset_return(0));
      return states;
    }
    std::tuple<States, double, bool, bool> step(VEC3 action)
    {
      VEC3 vel = VEC3::Zero();
      return this->step(std::pair<VEC3, VEC3>{ action, vel });
    }
    std::tuple<States, double, bool, bool> step(std::pair<VEC3, VEC3> action)
    {
      VEC3 tgt_pos = action.first;
      VEC3 tgt_vel = action.second;

      bool term = false, trunc = false;

      State cur_state = this->env.get_state();
      VEC3 cur_pos = cur_state.get_pos(), cur_vel = cur_state.get_vel();

      if ((cur_pos - tgt_pos).cwiseAbs().isZero(1e-9))
      {
        return { { cur_state }, 0.0, term, true };
      }
      Polynomial traj = calc_traj(cur_pos, cur_vel, tgt_pos, tgt_vel, get_traj_params());
      std::vector<State> observations{ cur_state };

      State u = State::Zero();
      int counter = -1;
      double t, dist;

      while (!(term || trunc))
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

        std::tuple<State, double, bool, bool> env_return = this->env.step(u);

        cur_state = std::get<0>(env_return);
        observations.push_back(cur_state);

        dist = (cur_state.get_pos() - tgt_pos).norm();
        if (dist < 0.5)
        {
          term = true;
        }
        else if (isnanf(dist))
        {
          trunc = true;
        }
      }
      return { observations, 0.0, term, trunc };
    }
  };

  class FifthOrderPolyPositionDroneEnv : public BasePolynomialPositionDroneEnv<polynomial::FifthOrderPolynomial>
  {
    double max_vel = 1.0;

   public:
    [[nodiscard]] double get_max_vel() const
    {
      return max_vel;
    }
    void set_max_vel(double max_vel)
    {
      this->max_vel = max_vel;
    }

   private:
    polynomial::FifthOrderPolynomial
    calc_traj(VEC3 pos, VEC3 vel, VEC3 tgt_pos, VEC3 tgt_vel, std::map<std::string, double> params) override;
    std::map<std::string, double> get_traj_params() override;

   public:
    using BasePolynomialPositionDroneEnv::BasePolynomialPositionDroneEnv;
    FifthOrderPolyPositionDroneEnv(double dt, data::State state, Eigen::Matrix<double, 4, 12> K, double max_vel)
        : max_vel(max_vel),
          BasePolynomialPositionDroneEnv(dt, state, K){};
  };

  class OptimalFifthOrderPolyPositionDroneEnv
      : public BasePolynomialPositionDroneEnv<polynomial::OptimalFifthOrderPolynomial>
  {
    double max_acc = 1.0;

   public:
    [[nodiscard]] double get_max_acc() const
    {
      return max_acc;
    }
    void set_max_acc(double max_acc)
    {
      this->max_acc = max_acc;
    }

   private:
    polynomial::OptimalFifthOrderPolynomial
    calc_traj(VEC3 pos, VEC3 vel, VEC3 tgt_pos, VEC3 tgt_vel, std::map<std::string, double> params) override;
    std::map<std::string, double> get_traj_params() override;

   public:
    using BasePolynomialPositionDroneEnv::BasePolynomialPositionDroneEnv;
    OptimalFifthOrderPolyPositionDroneEnv(double dt, data::State state, Eigen::Matrix<double, 4, 12> K, double max_acc)
        : max_acc(max_acc),
          BasePolynomialPositionDroneEnv(dt, state, K){};
  };
}  // namespace jdrones::envs

#endif  // ENVS_H
