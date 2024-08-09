/*
 * Copyright (c) 2024.  Jan-Hendrik Ewers
 * SPDX-License-Identifier: GPL-3.0-only
 */

#include "jdrones/polynomial.h"

#include <valarray>

namespace jdrones::polynomial
{

  VEC3 FifthOrderPolynomial::position(double t)
  {
    VEC3 a,b,c,d,e,f;
    a = coeffs.col(0);
    b = coeffs.col(1);
    c = coeffs.col(2);
    d = coeffs.col(3);
    e = coeffs.col(4);
    f = coeffs.col(5);
    return  a*std::pow(t,5) + b*std::pow(t,4) + c*std::pow(t,3) + d*std::pow(t,2) + e*t + f;

  }
  VEC3 FifthOrderPolynomial::velocity(double t)
  {
    VEC3 a,b,c,d,e;
    a = coeffs.col(0);
    b = coeffs.col(1);
    c = coeffs.col(2);
    d = coeffs.col(3);
    e = coeffs.col(4);
    return  5*a*std::pow(t,4) + 4*b*std::pow(t,3) + 3*c*std::pow(t,2) + 2*d*t + e;
  }
  VEC3 FifthOrderPolynomial::acceleration(double t)
  {
    VEC3 a,b,c,d;
    a = coeffs.col(0);
    b = coeffs.col(1);
    c = coeffs.col(2);
    d = coeffs.col(3);
    return  20*a*std::pow(t,3) + 12*b*std::pow(t,2) + 6*c*t + 2*d;
  }
  VEC3 FifthOrderPolynomial::jerk(double t)
  {
    VEC3 a,b,c;
    a = coeffs.col(0);
    b = coeffs.col(1);
    c = coeffs.col(2);
    return  60*a*std::pow(t,2) + 24*b*t + 6*c;
  }
  VEC3 FifthOrderPolynomial::snap(double t)
  {
    VEC3 a,b;
    a = coeffs.col(0);
    b = coeffs.col(1);
    return  120*a*t + 24*b;
  }

}  // namespace jdrones::polynomial
#include "jdrones/polynomial.h"
