/*
 * Copyright (c) 2024.  Jan-Hendrik Ewers
 * SPDX-License-Identifier: GPL-3.0-only
 */

#ifndef SOLVERS_H
#define SOLVERS_H

#include <eigen3/Eigen/Core>

#include <functional>

#include "jdrones/types.h"


namespace jdrones::solvers
{

  double bisection(std::function<double(double)> func, double a, double b, double tol = 1e-8, unsigned int max_iter = 10000);
  double bisection_with_right_expansion(
      std::function<double(double)> func,
      double a,
      double b,
      double tol = 1e-8,
      unsigned int max_iter = 10000);

  types::VEC2 quadratic_roots(types::VEC3 abc);
  types::VEC2 quadratic_roots(double a, double b, double c);
}  // namespace jdrones::solvers

#endif  // SOLVERS_H
