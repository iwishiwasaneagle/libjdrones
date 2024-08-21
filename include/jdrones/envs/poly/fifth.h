/*
 * Copyright (c) 2024.  Jan-Hendrik Ewers
 * SPDX-License-Identifier: GPL-3.0-only
 */

#ifndef FIFTH_H
#define FIFTH_H

#include <map>

#include <eigen3/Eigen/Core>
#include "jdrones/data.h"
#include "jdrones/polynomial.h"
#include "jdrones/envs/poly/base.h"

namespace jdrones::envs
{

using namespace jdrones::data;
  class FifthOrderPolyPositionDroneEnv : public BasePolynomialPositionDroneEnv<polynomial::FifthOrderPolynomial>
  {
    double max_vel = 1.0;
    polynomial::FifthOrderPolynomial
    calc_traj(VEC3 pos, VEC3 vel, VEC3 tgt_pos, VEC3 tgt_vel, std::map<std::string, double> params) override;
    std::map<std::string, double> get_traj_params() override;

   public:
    [[nodiscard]] double get_max_vel() const
    {
      return max_vel;
    }
    void set_max_vel(double max_vel)
    {
      this->max_vel = max_vel;
    }

    using BasePolynomialPositionDroneEnv::BasePolynomialPositionDroneEnv;
    FifthOrderPolyPositionDroneEnv(double dt, data::State state, double max_vel)
        : max_vel(max_vel),
          BasePolynomialPositionDroneEnv(dt, state){};
  };
}  // namespace jdrones::envs
#endif  // FIFTH_H
