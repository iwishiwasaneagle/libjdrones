/*
 * Copyright (c) 2024.  Jan-Hendrik Ewers
 * SPDX-License-Identifier: GPL-3.0-only
 */

#include "jdrones/solvers.h"

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
  while (fa * fb >= 0)
  {
    a = b;
    fa = fb;
    b = 2 * b;
    fb = func(b);

    if(iter++>max_iter)
    {
      break;
    }
  }
  return bisection(func, a, b, tol, max_iter - iter);
}

jdrones::types::VEC2 jdrones::solvers::quadratic_roots(jdrones::types::VEC3 abc)
{
  return quadratic_roots(abc(0), abc(1), abc(2));
}
jdrones::types::VEC2 jdrones::solvers::quadratic_roots(double a, double b, double c)
  {
    jdrones::types::VEC2 roots = jdrones::types::VEC2();
    double fourac = 4 * a * c;
    double b2 = b * b;
    if (abs(a) > 0 && b2 >= fourac)
    {
      roots(0) = (-b + sqrt(b2 - fourac)) / (2 * a);
      roots(1) = (-b - sqrt(b2 - fourac)) / (2 * a);
    }
    return roots;
  }
