/*
 * Copyright (c) 2024.  Jan-Hendrik Ewers
 * SPDX-License-Identifier: GPL-3.0-only
 */

#ifndef OPT_FIFTH_H
#define OPT_FIFTH_H
#include <map>

#include <eigen3/Eigen/Core>
#include "jdrones/envs/poly/base.h"
#include "jdrones/data.h"
#include "jdrones/polynomial.h"

namespace jdrones::envs
{
using namespace  jdrones::data;
  class OptimalFifthOrderPolyPositionDroneEnv
      : public BasePolynomialPositionDroneEnv<polynomial::OptimalFifthOrderPolynomial>
  {
    double max_acc = 1.0;
    polynomial::OptimalFifthOrderPolynomial
    calc_traj(VEC3 pos, VEC3 vel, VEC3 tgt_pos, VEC3 tgt_vel, std::map<std::string, double> params) override;
    std::map<std::string, double> get_traj_params() override;

   public:
    using BasePolynomialPositionDroneEnv::BasePolynomialPositionDroneEnv;
    OptimalFifthOrderPolyPositionDroneEnv(double dt, data::State state, double max_acc)
        : max_acc(max_acc),
          BasePolynomialPositionDroneEnv(dt, state){};

    [[nodiscard]] double get_max_acc() const
    {
      return max_acc;
    }
    void set_max_acc(double max_acc)
    {
      this->max_acc = max_acc;
    }
  };
}  // namespace jdrones::envs

#endif  // OPT_FIFTH_H
