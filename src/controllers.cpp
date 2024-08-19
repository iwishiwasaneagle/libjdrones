/*
 * Copyright (c) 2024.  Jan-Hendrik Ewers
 * SPDX-License-Identifier: GPL-3.0-only
 */

#include "jdrones/controllers.h"

#include <algorithm>

namespace jdrones::controllers
{
  double PIDController::calc_P()
  {
    return this->Kp * this->e;
  }
  double PIDController::calc_I()
  {
    this->integration += this->Ki * std::min(std::max(this->e * this->dt, -this->windup), this->windup);
    return this->integration;
  }
  double PIDController::calc_D()
  {
    return this->Kd * (this->e - this->prev_e) / this->dt;
  }
  void PIDController::reset()
  {
    this->integration = 0.0;
    this->e = 0.0;
    this->prev_e = this->e;
  }
  double PIDController::operator()(double measured, double setpoint)
  {
    this->e = error<double>(measured, setpoint);
    double p = calc_P(), i = calc_I(), d = calc_D();
    this->prev_e = this->e;
    return this->gain * (p + i + d);
  }


}  // namespace jdrones::controllers