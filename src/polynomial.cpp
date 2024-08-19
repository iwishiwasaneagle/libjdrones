/*
 * Copyright (c) 2024.  Jan-Hendrik Ewers
 * SPDX-License-Identifier: GPL-3.0-only
 */

#include "jdrones/polynomial.h"

namespace jdrones::polynomial
{
  /*****************
   * BasePolynomial *
   *****************/
  BMATRIX BasePolynomial::calc_b_matrix(VEC3 pos, VEC3 vel, VEC3 acc, VEC3 tgt_pos, VEC3 tgt_vel, VEC3 tgt_acc)
  {
    BMATRIX b_matrix;
    b_matrix.row(0) = pos;
    b_matrix.row(1) = tgt_pos;
    b_matrix.row(2) = vel;
    b_matrix.row(3) = tgt_vel;
    b_matrix.row(4) = acc;
    b_matrix.row(5) = tgt_acc;
    return b_matrix;
  }
  /***********************
   * FifthOrderPolynomial *
   ***********************/
  AMATRIX FifthOrderPolynomial::calc_A_matrix(double T)
  {
    const double T2 = T * T;
    const double T3 = T2 * T;
    const double T4 = T3 * T;
    const double T5 = T4 * T;
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
    const double t2 = t * t;
    const double t3 = t2 * t;
    const double t4 = t3 * t;
    const double t5 = t4 * t;
    VEC3 result = a * t5 + b * t4 + c * t3 + d * t2 + e * t + f;
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
    const double t2 = t * t;
    const double t3 = t2 * t;
    const double t4 = t3 * t;
    return a * t4 + b * t3 + c * t2 + d * t + e;
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
    const double t2 = t * t;
    const double t3 = t2 * t;
    return a * t3 + b * t2 + c * t + d;
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
    const double t2 = t * t;
    return a * t2 + b * t + c;
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
  void FifthOrderPolynomial::solve()
  {
    A_matrix = calc_A_matrix(this->T);
    const Eigen::FullPivHouseholderQR<AMATRIX> qr = A_matrix.fullPivHouseholderQr();
    this->coeffs.setZero();
    this->coeffs.col(0) = qr.solve(b_matrix.col(0));
    this->coeffs.col(1) = qr.solve(b_matrix.col(1));
    this->coeffs.col(2) = qr.solve(b_matrix.col(2));
  };

  /******************************
   * OptimalFifthOrderPolynomial *
   ******************************/
  void OptimalFifthOrderPolynomial::solve()
  {
    std::function const f = [this](double t_test) -> double
    { return get_global_max_abs_accelerations_from_time(this->b_matrix, t_test) - this->max_acc; };
    double t_optim = solvers::bisection_with_right_expansion(f, 0, this->T, this->tol, this->N);
    this->T = t_optim;
    this->FifthOrderPolynomial::solve();
  }

  VEC4 OptimalFifthOrderPolynomial::get_acceleration_at_jerk_0_single_dim(FifthOrderPolynomial *traj, unsigned int i)
  {
    COEFFMAT3 traj_coeffs = traj->get_coeffs();
    COEFFVEC jerk_coeffs = calc_jerk_coeffs(traj_coeffs, i);
    VEC4 jerk_0 = VEC4::Zero();
    jerk_0(0) = traj->get_T();

    jerk_0.block<2, 1>(0, 0) = jdrones::solvers::quadratic_roots(jerk_coeffs.block<3, 1>(0, 0));
    jerk_0 = jerk_0.cwiseMax(0).cwiseMin(traj->get_T());

    VEC4 ddx;
    for (int j = 0; j < 4; j++)
    {
      ddx(j) = acceleration(traj_coeffs, jerk_0(j))(i);
    }

    return ddx;
  }
  Eigen::Matrix<double, 4, 3> OptimalFifthOrderPolynomial::get_acceleration_at_jerk_zero(FifthOrderPolynomial *traj)
  {
    Eigen::Matrix<double, 4, 3> ddx;
    ddx.setZero();
    ddx.col(0) = get_acceleration_at_jerk_0_single_dim(traj, 0);
    ddx.col(1) = get_acceleration_at_jerk_0_single_dim(traj, 1);
    ddx.col(2) = get_acceleration_at_jerk_0_single_dim(traj, 2);
    return ddx;
  }
  VEC3 OptimalFifthOrderPolynomial::get_max_abs_acceleration(FifthOrderPolynomial *traj)
  {
    Eigen::Matrix<double, 4, 3> acceleration = get_acceleration_at_jerk_zero(traj);
    return acceleration.cwiseAbs().colwise().maxCoeff();
  }
  VEC3 OptimalFifthOrderPolynomial::get_max_abs_accelerations_from_time(BMATRIX b, double t)
  {
    if (t <= 0)
    {
      return VEC3::Constant(INFINITY);
    }
    FifthOrderPolynomial traj(b, t);
    traj.solve();
    return get_max_abs_acceleration(&traj);
  }
  double OptimalFifthOrderPolynomial::get_global_max_abs_accelerations_from_time(BMATRIX b, double t)
  {
    VEC3 max_abs_acceleration = get_max_abs_accelerations_from_time(b, t);
    double global_max_abs_acceleration = max_abs_acceleration.maxCoeff();
    return global_max_abs_acceleration;
  }

}  // namespace jdrones::polynomial
#include "jdrones/polynomial.h"
