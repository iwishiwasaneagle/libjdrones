/*
 * Copyright (c) 2024.  Jan-Hendrik Ewers
 * SPDX-License-Identifier: GPL-3.0-only
 */

#ifndef CONTROLLERS_H
#define CONTROLLERS_H

#include <eigen3/Eigen/Eigen>

#include "jdrones/solvers.h"

namespace jdrones::controllers
{
  template<int xdim, int udim>
  class LQRController
  {
    Eigen::Matrix<double, xdim, 1> e;
    Eigen::Matrix<double, udim, xdim> K;

   public:
    [[nodiscard]] Eigen::Matrix<double, xdim, 1> get_e() const
    {
      return e;
    }

    static Eigen::Matrix<double, xdim, 1> error(
        Eigen::Matrix<double, xdim, 1> measured,
        Eigen::Matrix<double, xdim, 1> setpoint)
    {
      return setpoint - measured;
    }

    LQRController(Eigen::Matrix<double, udim, xdim> K) : K(K), e(Eigen::Matrix<double, xdim, 1>::Zero())
    {
    }
    LQRController(
        Eigen::Matrix<double, xdim, xdim> A,
        Eigen::Matrix<double, xdim, udim> B,
        Eigen::Matrix<double, xdim, xdim> Q,
        Eigen::Matrix<double, udim, udim> R)
        : e(Eigen::Matrix<double, xdim, 1>::Zero()),
          K(Eigen::Matrix<double, udim, xdim>::Zero())
    {
      Eigen::Matrix<double, xdim, xdim> P = Eigen::Matrix<double, xdim, xdim>::Zero();
      bool success = solvers::solveRiccatiArimotoPotter<xdim, udim>(A, B, Q, R, P);
      if (!success)
      {
        throw "Failed to solve CARE";
      }
      K = R.inverse() * (B.transpose() * P);
    }

    Eigen::Matrix<double, udim, xdim> get_K()
    {
      return this->K;
    }
    void reset()
    {
      this->e.setZero();
    }
    Eigen::Matrix<double, udim, 1> operator()(
        Eigen::Matrix<double, xdim, 1> measured,
        Eigen::Matrix<double, xdim, 1> setpoint)
    {
      this->e = error(measured, setpoint);
      return this->K * this->e;
    }
  };

}  // namespace jdrones::controllers

#endif  // CONTROLLERS_H
