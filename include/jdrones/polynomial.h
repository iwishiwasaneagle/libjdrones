/*
 * Copyright (c) 2024.  Jan-Hendrik Ewers
 * SPDX-License-Identifier: GPL-3.0-only
 */

#ifndef POLYNOMIAL_H
#define POLYNOMIAL_H
#include <cmath>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

#include "data.h"
#include "jdrones/solvers.h"

namespace jdrones::polynomial
{
  using namespace jdrones::types;

  using BMATRIX = Eigen::Matrix<double, 6, 3>;
  using AMATRIX = Eigen::Matrix<double, 6, 6>;
  using COEFFVEC = Eigen::Matrix<double, 6, 1>;
  using COEFFMAT3 = Eigen::Matrix<double, 6, 3>;

  class BasePolynomial
  {
   protected:
    BMATRIX b_matrix;
    static BMATRIX calc_b_matrix(VEC3 pos, VEC3 vel, VEC3 acc, VEC3 tgt_pos, VEC3 tgt_vel, VEC3 tgt_acc)
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

    double T;

   public:
    BasePolynomial(BMATRIX b_matrix, double T) : b_matrix(b_matrix), T(T)
    {
    }

    BasePolynomial(VEC3 pos, VEC3 vel, VEC3 acc, VEC3 tgt_pos, VEC3 tgt_vel, VEC3 tgt_acc, double T)
        : T(T),
          b_matrix(calc_b_matrix(pos, vel, acc, tgt_pos, tgt_vel, tgt_acc))
    {
    }

    double get_T()
    {
      return T;
    };

    virtual VEC3 position(double t) = 0;
    virtual VEC3 velocity(double t) = 0;
    virtual VEC3 acceleration(double t) = 0;

    BMATRIX get_b_matrix()
    {
      return b_matrix;
    }
    virtual void solve() = 0;
  };

  class FifthOrderPolynomial : public BasePolynomial
  {
   protected:
    AMATRIX A_matrix;
    static AMATRIX calc_A_matrix(double T);

    COEFFMAT3 coeffs;

   public:
    FifthOrderPolynomial(BMATRIX b_matrix, double T) : BasePolynomial(b_matrix, T){};

    FifthOrderPolynomial(VEC3 pos, VEC3 vel, VEC3 acc, VEC3 tgt_pos, VEC3 tgt_vel, VEC3 tgt_acc, double T)
        : BasePolynomial(pos, vel, acc, tgt_pos, tgt_vel, tgt_acc, T)
    {
    }
    static VEC3 position(COEFFMAT3 coeffs, double t);
    VEC3 position(double t) override;
    static VEC3 velocity(COEFFMAT3 coeffs, double t);
    VEC3 velocity(double t) override;
    static VEC3 acceleration(COEFFMAT3 coeffs, double t);
    VEC3 acceleration(double t) override;
    static VEC3 jerk(COEFFMAT3 coeffs, double t);
    VEC3 jerk(double t);
    static VEC3 snap(COEFFMAT3 coeffs, double t);
    VEC3 snap(double t);

    static COEFFVEC calc_snap_coeffs(COEFFMAT3 traj, unsigned int i);
    static COEFFMAT3 calc_snap_coeffs(COEFFMAT3 traj);
    static COEFFMAT3 calc_snap_coeffs(FifthOrderPolynomial traj);
    static COEFFVEC calc_jerk_coeffs(COEFFMAT3 traj, unsigned int i);
    static COEFFMAT3 calc_jerk_coeffs(COEFFMAT3 traj);
    static COEFFMAT3 calc_jerk_coeffs(FifthOrderPolynomial traj);
    static COEFFVEC calc_acceleration_coeffs(COEFFMAT3 traj, unsigned int i);
    static COEFFMAT3 calc_acceleration_coeffs(COEFFMAT3 traj);
    static COEFFMAT3 calc_acceleration_coeffs(FifthOrderPolynomial traj);
    static COEFFVEC calc_velocity_coeffs(COEFFMAT3 traj, unsigned int i);
    static COEFFMAT3 calc_velocity_coeffs(COEFFMAT3 traj);
    static COEFFMAT3 calc_velocity_coeffs(FifthOrderPolynomial traj);

    Eigen::Matrix<double, 6, 3> get_coeffs()
    {
      return this->coeffs;
    }

    void solve() override
    {
      A_matrix = calc_A_matrix(this->T);
      const Eigen::FullPivHouseholderQR<AMATRIX> qr = A_matrix.fullPivHouseholderQr();
      this->coeffs.setZero();
      this->coeffs.col(0) = qr.solve(b_matrix.col(0));
      this->coeffs.col(1) = qr.solve(b_matrix.col(1));
      this->coeffs.col(2) = qr.solve(b_matrix.col(2));
    }
  };

  class OptimalFifthOrderPolynomial : public FifthOrderPolynomial
  {
   protected:
    float max_acc;
    float tol;
    unsigned int N;

   public:
    [[nodiscard]] double get_max_acc()
    {
      return max_acc;
    }

    static VEC4 get_acceleration_at_jerk_0_single_dim(FifthOrderPolynomial *traj, unsigned int i)
    {
      COEFFMAT3 traj_coeffs = traj->get_coeffs();
      COEFFVEC jerk_coeffs = calc_jerk_coeffs(traj_coeffs, i);
      VEC4 jerk_0 = VEC4::Zero();
      jerk_0(0) = traj->get_T();

      jerk_0.block<2, 1>(0, 0) = jdrones::solvers::quadratic_roots(jerk_coeffs.block<3, 1>(0, 0));
      jerk_0 = jerk_0.cwiseMax(0).cwiseMin( traj->get_T());

      VEC4 ddx;
      for (int j = 0; j < 4; j++)
      {
        ddx(j) = acceleration(traj_coeffs, jerk_0(j))(i);
      }

      return ddx;
    }

    static Eigen::Matrix<double, 4, 3> get_acceleration_at_jerk_zero(FifthOrderPolynomial *traj)
    {
      Eigen::Matrix<double, 4, 3> ddx;
      ddx.setZero();
      ddx.col(0) = get_acceleration_at_jerk_0_single_dim(traj, 0);
      ddx.col(1) = get_acceleration_at_jerk_0_single_dim(traj, 1);
      ddx.col(2) = get_acceleration_at_jerk_0_single_dim(traj, 2);
      return ddx;
    }
    static VEC3 get_max_abs_acceleration(FifthOrderPolynomial *traj)
    {
      Eigen::Matrix<double, 4, 3> acceleration = get_acceleration_at_jerk_zero(traj);
      return acceleration.cwiseAbs().colwise().maxCoeff();
    }
    static VEC3 get_max_abs_accelerations_from_time(BMATRIX b, double t)
    {
      if (t <= 0)
      {
        return VEC3::Constant(INFINITY);
      }
      if (b.isZero(0))
      {
        return VEC3::Zero();
      }
      FifthOrderPolynomial traj(b, t);
      traj.solve();
      return get_max_abs_acceleration(&traj);
    }
    static double get_global_max_abs_accelerations_from_time(BMATRIX b, double t)
    {
      VEC3 max_abs_acceleration = get_max_abs_accelerations_from_time(b, t);
      double global_max_abs_acceleration = max_abs_acceleration.maxCoeff();
      return global_max_abs_acceleration;
    }
    OptimalFifthOrderPolynomial(
        BMATRIX b_matrix,
        double T = 10.0,
        double max_acc = 1.0,
        double tol = 1e-8,
        unsigned int N = 1000)
        : FifthOrderPolynomial(b_matrix, T),
          max_acc(max_acc),
          tol(tol),
          N(N){};

    OptimalFifthOrderPolynomial(
        VEC3 pos,
        VEC3 vel,
        VEC3 acc,
        VEC3 tgt_pos,
        VEC3 tgt_vel,
        VEC3 tgt_acc,
        double T = 10.0,
        double max_acc = 1.0,
        double tol = 1e-8,
        unsigned int N = 1000)
        : FifthOrderPolynomial(pos, vel, acc, tgt_pos, tgt_vel, tgt_acc, T),
          max_acc(max_acc),
          tol(tol),
          N(N)
    {
    }

    void solve() override;
  };
}  // namespace jdrones::polynomial

#endif  // POLYNOMIAL_H
