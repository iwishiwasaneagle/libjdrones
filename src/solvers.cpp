/*
 * Copyright (c) 2024.  Jan-Hendrik Ewers
 * SPDX-License-Identifier: GPL-3.0-only
 */

#include "jdrones/solvers.h"

#include <math.h>

double jdrones::solvers::bisection(std::function<double(double)> func, double a, double b, double tol, unsigned int max_iter)
{
  double fa, fb, fc;

  fa = func(a);
  fb = func(b);
  if (fa * fb >= 0)
  {
    throw "Sign of f(a) and f(b) are the same";
  }

  double c = a;
  unsigned int iter = 0;
  while ((b - a) >= tol && iter++ < max_iter)
  {
    // Find middle point
    c = (a + b) / 2;

    fc = func(c);

    // Check if middle point is root
    if (fc == 0.0)
    {
      break;
    }

    // Decide the side to repeat the steps
    if (fc * fa < 0)
    {
      b = c;
      fb = fc;
    }
    else
    {
      a = c;
      fa = fc;
    }
  }
  return c;
}
double jdrones::solvers::bisection_with_right_expansion(
    std::function<double(double)> func,
    double a,
    double b,
    double tol,
    unsigned int max_iter)
{
  double fa, fb;
  unsigned int iter = 0;
  fa = func(a);
  fb = func(b);
  while (fa * fb >= 0 && iter++ < max_iter)
  {
    a = b;
    fa = fb;
    b = 2 * b;
    fb = func(b);
  }
  return bisection(func, a, b, tol, max_iter - iter);
}
