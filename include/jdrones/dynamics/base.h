/*
 * Copyright (c) 2024.  Jan-Hendrik Ewers
 * SPDX-License-Identifier: GPL-3.0-only
 */

#ifndef DYNAMICS_BASE_H
#define DYNAMICS_BASE_H
#include <eigen3/Eigen/Eigen>

#include "jdrones/constants.h"
#include "jdrones/data.h"
#include "jdrones/gymnasium.h"
#include "jdrones/transforms.h"

namespace jdrones::dynamics
{
using namespace jdrones::data;
  class BaseDynamicModelDroneEnv : public jdrones::gymnasium::Env<State, State, VEC4>
  {
   public:
    BaseDynamicModelDroneEnv(double dt, State state) : dt(dt), state(state)
    {
    }
    explicit BaseDynamicModelDroneEnv(double dt) : BaseDynamicModelDroneEnv(dt, State::Zero()){};
    std::tuple<State, std::map<std::string, Eigen::VectorXd>> reset(State state) override
    {
      this->state = state;
      this->state.set_quat(euler_to_quat(this->state.get_rpy()));
      return { this->state, {} };
    };
    std::tuple<State, std::map<std::string, Eigen::VectorXd>> reset() override
    {
      return this->reset(State::Zero());
    };
    std::tuple<State, double, bool, bool, std::map<std::string, Eigen::VectorXd>> step(VEC4) override;

    const double dt;

    [[nodiscard]] double get_dt() const
    {
      return dt;
    }
    [[nodiscard]] double get_l() const
    {
      return l;
    }
    [[nodiscard]] double get_k_t() const
    {
      return k_T;
    }
    [[nodiscard]] double get_k_q() const
    {
      return k_Q;
    }
    [[nodiscard]] double get_tau_t() const
    {
      return tau_T;
    }
    [[nodiscard]] double get_tau_q() const
    {
      return tau_Q;
    }
    [[nodiscard]] Eigen::Vector3d get_drag_coeffs() const
    {
      return drag_coeffs;
    }
    [[nodiscard]] double get_mass() const
    {
      return mass;
    }
    [[nodiscard]] Eigen::Vector3d get_i() const
    {
      return I;
    }
    [[nodiscard]] Eigen::Matrix<double, 4, 4> get_mixing_matrix() const
    {
      return mixing_matrix;
    }
    [[nodiscard]] Eigen::Matrix<double, 12, 12> get_A() const
    {
      return A;
    }
    [[nodiscard]] Eigen::Matrix<double, 12, 4> get_B() const
    {
      return B;
    }
    [[nodiscard]] Eigen::Matrix<double, 12, 1> get_C() const
    {
      return C;
    }
    [[nodiscard]] State get_state() const
    {
      return state;
    }

    const double l = 0.1;
    const double k_T = 0.1;
    const double k_Q = 0.05;
    const double tau_T = 0.1;
    const double tau_Q = 0.1;
    const Eigen::Vector3d drag_coeffs{ 0.1, 0.1, 0.1 };
    const double mass = 1.4;
    const Eigen::Vector3d I{ 0.1, 0.1, 0.1 };
    const Eigen::Matrix<double, 4, 4> mixing_matrix{ { 0, -l* k_T, 0., l* k_T },
                                                     { -l * k_T, 0, l* k_T, 0. },
                                                     { k_Q, -k_Q, k_Q, -k_Q },
                                                     { k_T, k_T, k_T, k_T } };

    const Eigen::Matrix<double, 12, 12> A{
      { 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0 },              // x
      { 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0 },              // y
      { 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0 },              // z
      { 0, 0, 0, 0, 0, 0, 0, constants::g, 0, 0, 0, 0 },   // dx
      { 0, 0, 0, 0, 0, 0, -constants::g, 0, 0, 0, 0, 0 },  // dy
      { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },              // dz
      { 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0 },              // phi
      { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0 },              // theta
      { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1 },              // psi
      { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },              // dphi
      { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },              // dtheta
      { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }               // dpsi
    };

    const Eigen::Matrix<double, 12, 4> B{
      { 0, 0, 0, 0 },         // x
      { 0, 0, 0, 0 },         // y
      { 0, 0, 0, 0 },         // z
      { 0, 0, 0, 0 },         // dx
      { 0, 0, 0, 0 },         // dy
      { 0, 0, 0, 1 / mass },  // dz
      { 0, 0, 0, 0 },         // phi
      { 0, 0, 0, 0 },         // theta
      { 0, 0, 0, 0 },         // psi
      { 1 / I(0), 0, 0, 0 },  // dphi
      { 0, 1 / I(1), 0, 0 },  // dtheta
      { 0, 0, 1 / I(2), 0 }   // dpsi
    };

    const Eigen::Matrix<double, 12, 1> C{ { 0, 0, 0, 0, 0, -constants::g, 0, 0, 0, 0, 0, 0 } };

    VEC4 rpm2rpyT(VEC4 rpm);
    VEC4 rpyT2rpm(VEC4 rpyT);

   protected:
    State state;
    virtual State calc_dstate(VEC4) = 0;
  };
}  // namespace jdrones::dynamics
#endif  // DYNAMICS_BASE_H
