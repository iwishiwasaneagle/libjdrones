/*
 * Copyright (c) 2024.  Jan-Hendrik Ewers
 * SPDX-License-Identifier: GPL-3.0-only
 */

#include "jdrones/polynomial.h"

#include "jdrones/data.h"

namespace jdrones::polynomial
{
  AMATRIX FifthOrderPolynomial::calc_A_matrix(double T)
  {
    double T2, T3, T4, T5;
    T2 = std::pow(T, 2);
    T3 = std::pow(T, 3);
    T4 = std::pow(T, 4);
    T5 = std::pow(T, 5);
    AMATRIX A{
      { 0.0, 0.0, 0.0, 0.0, 0.0, 1.0 },             // f(t=0)
      { T5, T4, T3, T2, T, 1.0 },                   // f(t=T)
      { 0.0, 0.0, 0.0, 0.0, 1.0, 0.0 },             // f'(t=0)
      { 5 * T4, 4 * T3, 3 * T2, 2 * T, 1.0, 0.0 },  // f'(t=T)
      { 0.0, 0.0, 0.0, 2.0, 0.0, 0.0 },             // f''(t=0)
      { 20 * T3, 12 * T2, 6 * T, 2, 0.0, 0.0 }      // f''(t=T)
    };
    return A;
  }
  VEC3 FifthOrderPolynomial::position(COEFFMAT3 coeffs, double t)
  {
    VEC3 a, b, c, d, e, f;
    a = coeffs.row(0);
    b = coeffs.row(1);
    c = coeffs.row(2);
    d = coeffs.row(3);
    e = coeffs.row(4);
    f = coeffs.row(5);
    VEC3 result = a * std::pow(t, 5) + b * std::pow(t, 4) + c * std::pow(t, 3) + d * std::pow(t, 2) + e * t + f;
    return result;
  }

  VEC3 FifthOrderPolynomial::position(double t)
  {
    return position(this->coeffs, t);
  }
  VEC3 FifthOrderPolynomial::velocity(COEFFMAT3 coeffs, double t)
  {
    COEFFMAT3 vel_coeffs = calc_velocity_coeffs(coeffs);
    VEC3 a, b, c, d, e;
    a = vel_coeffs.row(0);
    b = vel_coeffs.row(1);
    c = vel_coeffs.row(2);
    d = vel_coeffs.row(3);
    e = vel_coeffs.row(4);
    return a * std::pow(t, 4) + b * std::pow(t, 3) + c * std::pow(t, 2) + d * t + e;
  }
  VEC3 FifthOrderPolynomial::velocity(double t)
  {
    return velocity(this->coeffs, t);
  }
  VEC3 FifthOrderPolynomial::acceleration(COEFFMAT3 coeffs, double t)
  {
    COEFFMAT3 acc_coeffs = calc_acceleration_coeffs(coeffs);
    VEC3 a, b, c, d;
    a = acc_coeffs.row(0);
    b = acc_coeffs.row(1);
    c = acc_coeffs.row(2);
    d = acc_coeffs.row(3);
    return a * std::pow(t, 3) + b * std::pow(t, 2) + c * t + d;
  }
  VEC3 FifthOrderPolynomial::acceleration(double t)
  {
    return acceleration(this->coeffs, t);
  }
  VEC3 FifthOrderPolynomial::jerk(COEFFMAT3 coeffs, double t)
  {
    COEFFMAT3 jerk_coeffs = calc_jerk_coeffs(coeffs);
    VEC3 a, b, c;
    a = jerk_coeffs.row(0);
    b = jerk_coeffs.row(1);
    c = jerk_coeffs.row(2);
    return a * std::pow(t, 2) + b * t + c;
  }
  VEC3 FifthOrderPolynomial::jerk(double t)
  {
    return jerk(this->coeffs, t);
  }
  VEC3 FifthOrderPolynomial::snap(COEFFMAT3 coeffs, double t)
  {
    COEFFMAT3 snap_coeffs = calc_snap_coeffs(coeffs);
    VEC3 a, b;
    a = snap_coeffs.row(0);
    b = snap_coeffs.row(1);
    return a * t + b;
  }
  VEC3 FifthOrderPolynomial::snap(double t)
  {
    return snap(this->coeffs, t);
  }
  COEFFVEC FifthOrderPolynomial::calc_snap_coeffs(COEFFMAT3 traj, unsigned int i)
  {
    COEFFVEC traj_coeffs = traj.col(i);
    COEFFVEC snap_coeffs{ 120 * traj_coeffs(0), 24 * traj_coeffs(1), 0, 0, 0, 0 };
    return snap_coeffs;
  }
  COEFFMAT3 FifthOrderPolynomial::calc_snap_coeffs(COEFFMAT3 traj)
  {
    COEFFMAT3 snap_coeffs;
    snap_coeffs.col(0) = calc_snap_coeffs(traj, 0);
    snap_coeffs.col(1) = calc_snap_coeffs(traj, 1);
    snap_coeffs.col(2) = calc_snap_coeffs(traj, 2);
    return snap_coeffs;
  }
  COEFFMAT3 FifthOrderPolynomial::calc_snap_coeffs(FifthOrderPolynomial traj)
  {
    return calc_acceleration_coeffs(traj.get_coeffs());
  }
  COEFFVEC FifthOrderPolynomial::calc_jerk_coeffs(COEFFMAT3 traj, unsigned int i)
  {
    COEFFVEC traj_coeffs = traj.col(i);
    COEFFVEC jerk_coeffs{ 60 * traj_coeffs(0), 24 * traj_coeffs(1), 6 * traj_coeffs(2), 0, 0, 0 };
    return jerk_coeffs;
  }
  COEFFMAT3 FifthOrderPolynomial::calc_jerk_coeffs(COEFFMAT3 traj)
  {
    COEFFMAT3 jerk_coeffs;
    jerk_coeffs.col(0) = calc_jerk_coeffs(traj, 0);
    jerk_coeffs.col(1) = calc_jerk_coeffs(traj, 1);
    jerk_coeffs.col(2) = calc_jerk_coeffs(traj, 2);
    return jerk_coeffs;
  }
  COEFFMAT3 FifthOrderPolynomial::calc_jerk_coeffs(FifthOrderPolynomial traj)
  {
    return calc_acceleration_coeffs(traj.get_coeffs());
  }

  COEFFVEC FifthOrderPolynomial::calc_acceleration_coeffs(COEFFMAT3 traj, unsigned int i)
  {
    COEFFVEC traj_coeffs = traj.col(i);
    COEFFVEC acceleration_coeffs{ 20 * traj_coeffs(0), 12 * traj_coeffs(1), 6 * traj_coeffs(2), 2 * traj_coeffs(3), 0, 0 };
    return acceleration_coeffs;
  }
  COEFFMAT3 FifthOrderPolynomial::calc_acceleration_coeffs(COEFFMAT3 traj)
  {
    COEFFMAT3 acceleration_coeffs;
    acceleration_coeffs.col(0) = calc_acceleration_coeffs(traj, 0);
    acceleration_coeffs.col(1) = calc_acceleration_coeffs(traj, 1);
    acceleration_coeffs.col(2) = calc_acceleration_coeffs(traj, 2);
    return acceleration_coeffs;
  }
  COEFFMAT3 FifthOrderPolynomial::calc_acceleration_coeffs(FifthOrderPolynomial traj)
  {
    return calc_acceleration_coeffs(traj.get_coeffs());
  }
  COEFFVEC FifthOrderPolynomial::calc_velocity_coeffs(COEFFMAT3 traj, unsigned int i)
  {
    COEFFVEC traj_coeffs = traj.col(i);
    COEFFVEC velocity_coeffs{ 5 * traj_coeffs(0), 4 * traj_coeffs(1), 3 * traj_coeffs(2),
                              2 * traj_coeffs(3), traj_coeffs(4),     0 };
    return velocity_coeffs;
  }
  COEFFMAT3 FifthOrderPolynomial::calc_velocity_coeffs(COEFFMAT3 traj)
  {
    COEFFMAT3 velocity_coeffs;
    velocity_coeffs.col(0) = calc_velocity_coeffs(traj, 0);
    velocity_coeffs.col(1) = calc_velocity_coeffs(traj, 1);
    velocity_coeffs.col(2) = calc_velocity_coeffs(traj, 2);
    return velocity_coeffs;
  }
  COEFFMAT3 FifthOrderPolynomial::calc_velocity_coeffs(FifthOrderPolynomial traj)
  {
    return calc_velocity_coeffs(traj.get_coeffs());
  }

  void OptimalFifthOrderPolynomial::solve()
  {
    std::function const f = [this](double t_test) -> double
    { return get_global_max_abs_accelerations_from_time(this->b_matrix, t_test) - this->max_acc; };
    double t_optim = solvers::bisection_with_right_expansion(f, 0, this->T, this->tol, this->N);
    this->T = t_optim;
    this->FifthOrderPolynomial::solve();
  }

}  // namespace jdrones::polynomial
#include "jdrones/polynomial.h"
