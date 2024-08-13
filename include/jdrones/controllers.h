/*
 * Copyright (c) 2024.  Jan-Hendrik Ewers
 * SPDX-License-Identifier: GPL-3.0-only
 */

#ifndef CONTROLLERS_H
#define CONTROLLERS_H
#include "jdrones/data.h"

namespace jdrones::controllers
{
  template <typename T>
  T error(T measured, T setpoint)
  {
    return measured - setpoint;
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

  class LQRController
  {
   private:
    data::X e;
    Eigen::Matrix<double, 4, 12> K;

   public:
    LQRController(Eigen::Matrix<double, 4, 12> K) : K(K), e(data::X::Zero())
    {
    }

    VEC4 operator()(data::State measured, data::State setpoint);
    VEC4 operator()(data::X measured, data::X setpoint);
    void set_K(Eigen::Matrix<double, 4, 12> K)
    {
      this->K = K;
    }
    Eigen::Matrix<double, 4, 12> get_K()
    {
      return this->K;
    }
    void reset();
  };

}  // namespace jdrones::controllers

#endif  // CONTROLLERS_H
