/*
 * Copyright (c) 2024.  Jan-Hendrik Ewers
 * SPDX-License-Identifier: GPL-3.0-only
 */

#ifndef CONTROLLERS_H
#define CONTROLLERS_H
#include "jdrones/data.h"
#include "solvers.h"

namespace jdrones::controllers
{
  template<typename T>
  T error(T measured, T setpoint)
  {
    return setpoint - measured;
  }

  class PIDController
  {
   private:
    double Kp, Ki, Kd, dt, gain, windup;
    double prev_e, e, integration;
    double calc_P();
    double calc_I();
    double calc_D();

   public:
    PIDController(double Kp, double Ki, double Kd, double dt, double gain, double windup)
        : Kp(Kp),
          Ki(Ki),
          Kd(Kd),
          dt(dt),
          gain(gain),
          windup(windup)
    {
      this->PIDController::reset();
    };
    double operator()(double measured, double setpoint);
    void reset();
  };

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

    LQRController(Eigen::Matrix<double, udim, xdim> K) : K(K), e(Eigen::Matrix<double, xdim, 1>::Zero())
    {
    }
    LQRController(
      Eigen::Matrix<double, xdim, xdim> A,
      Eigen::Matrix<double, xdim, udim> B,
      Eigen::Matrix<double, xdim, xdim> Q,
      Eigen::Matrix<double, udim, udim> R
) :e(Eigen::Matrix<double, xdim, 1>::Zero()), K(Eigen::Matrix<double, udim, xdim>::Zero())
    {
      Eigen::Matrix<double, xdim, xdim> P = Eigen::Matrix<double, xdim, xdim>::Zero();
      bool success = solvers::solveRiccatiArimotoPotter<xdim, udim>(A, B, Q, R, P);
      if(!success)
      {
        throw "Failed to solve CARE";
      }
      K = R.inverse() * (B.transpose() * P);
    }

    void set_K(Eigen::Matrix<double, udim, xdim> K)
    {
      this->K = K;
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
      this->e = error<Eigen::Matrix<double, xdim, 1>>(measured, setpoint);
      return this->K * this->e;
    }
  };

}  // namespace jdrones::controllers

#endif  // CONTROLLERS_H
