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
#include "solvers.h"

namespace jdrones::polynomial
{
  using VEC3 = Eigen::Matrix<double, 3, 1>;

  class BasePolynomial
  {
   protected:
    Eigen::Matrix<double, 6, 3> b_matrix;
    static Eigen::Matrix<double, 6, 3> calc_b_matrix(VEC3 pos, VEC3 vel, VEC3 acc, VEC3 tgt_pos, VEC3 tgt_vel, VEC3 tgt_acc)
    {
      Eigen::Matrix<double, 6, 3> b_matrix;
      b_matrix.row(0) = pos;
      b_matrix.row(2) = vel;
      b_matrix.row(4) = acc;
      b_matrix.row(1) = tgt_pos;
      b_matrix.row(3) = tgt_vel;
      b_matrix.row(5) = tgt_acc;
      return b_matrix;
    }

    double T;

   public:
    BasePolynomial(Eigen::Matrix<double, 6, 3> b_matrix, double T) : b_matrix(b_matrix), T(T)
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

    virtual Eigen::Matrix3Xd get_coeffs() = 0;
  };

  class FifthOrderPolynomial : public BasePolynomial
  {
   protected:
    Eigen::Matrix<double, 6, 6> A_matrix;
    static Eigen::Matrix<double, 6, 6> calc_A_matrix(double T)
    {
      double T2, T3, T4, T5;
      T2 = std::pow(T, 2);
      T3 = std::pow(T, 3);
      T4 = std::pow(T, 4);
      T5 = std::pow(T, 5);
      Eigen::Matrix<double, 6, 6> A = Eigen::Matrix<double, 6, 6>::Zero();
      A.row(0) << 0.0, 0.0, 0.0, 0.0, 0.0, 1.0;  // f(t=0)
      A.row(1) << T5, T4, T3, T2, T,
          1.0;                                   // # f(t=T)
      A.row(2) << 0.0, 0.0, 0.0, 0.0, 1.0, 0.0;  // f'(t=0)
      A.row(3) << 5 * T4, 4 * T3, 3 * T2, 2 * T, 1.0,
          0.0;                                           // f'(t=T)
      A.row(4) << 0.0, 0.0, 0.0, 2.0, 0.0, 0.0;          // f''(t=0)
      A.row(5) << 20 * T3, 12 * T2, 6 * T, 2, 0.0, 0.0;  // f''(t=T)

      return A;
    }

    Eigen::Matrix<double, 3, 6> coeffs;

   public:
    FifthOrderPolynomial(
        VEC3 pos,
        VEC3 vel,
        VEC3 acc,
        VEC3 tgt_pos,
        VEC3 tgt_vel,
        VEC3 tgt_acc,
        double T,
        bool _solve = true)
        : BasePolynomial(pos, vel, acc, tgt_pos, tgt_vel, tgt_acc, T)
    {
      if (_solve)
      {
        solve();
      }
    }
    FifthOrderPolynomial(Eigen::Matrix<double, 6, 3> b_matrix, double T, bool _solve = true) : BasePolynomial(b_matrix, T)
    {
      if (_solve)
      {
        solve();
      }
    };
    VEC3 position(double t) override;
    VEC3 velocity(double t) override;
    VEC3 acceleration(double t) override;
    VEC3 jerk(double t);
    VEC3 snap(double t);

    Eigen::Matrix<double, 3, -1> get_coeffs() override
    {
      return this->coeffs;
    }

    FifthOrderPolynomial solve()
    {
      A_matrix = calc_A_matrix(this->T);
      const Eigen::FullPivHouseholderQR<Eigen::Matrix<double, 6, 6>> qr = A_matrix.fullPivHouseholderQr();
      this->coeffs = Eigen::Matrix<double, 3, 6>::Zero();
      this->coeffs.row(0) = qr.solve(b_matrix.col(0));
      this->coeffs.row(1) = qr.solve(b_matrix.col(1));
      this->coeffs.row(2) = qr.solve(b_matrix.col(2));
      return *this;
    }
  };

  class OptimalFifthOrderPolynomial : public FifthOrderPolynomial
  {
   protected:
    float max_acc;
    float tmax;
    float tol;
    unsigned int N;

    static Eigen::Matrix<double, 3, 4> get_acceleration_at_jerk_zero(FifthOrderPolynomial traj, double tmax)
    {
      Eigen::Matrix<double, 3, 6> jerk_coeffs;
      jerk_coeffs.setZero();
      Eigen::Matrix<double, 3, 6> traj_coeffs = traj.get_coeffs();
      jerk_coeffs.col(0) = 60 * traj_coeffs.col(0);
      jerk_coeffs.col(1) = 24 * traj_coeffs.col(0);
      jerk_coeffs.col(2) = 6 * traj_coeffs.col(0);

      VEC3 a = (jerk_coeffs.col(1).array().pow(2) - 4 * (jerk_coeffs.col(0).array() * jerk_coeffs.col(2).array())).sqrt();
      VEC3 b = 2 * jerk_coeffs.col(0);

      Eigen::Matrix<double, 3, 4> ddx;
      Eigen::Matrix<double, 1, 4> jerk_0, x, y, w;
      double z;
      for (int i = 0; i < 3; i++)
      {
        if (b(0) == 0)
        {
          jerk_0.setZero();
          jerk_0 << tmax;
        }
        else
        {
          jerk_0 << -2 * jerk_coeffs(i, 0) + a(i) / b(i), -2 * jerk_coeffs(i, 0) + a(i) / b(i), tmax, 0.0;
          jerk_0 = jerk_0.cwiseMin(0).cwiseMax(tmax);
        }
        x = 20.0 * traj_coeffs(i, 0) * jerk_0.array().pow(3);
        y = 12 * traj_coeffs(i, 1) * jerk_0.array().pow(2);
        w = 6 * (traj_coeffs(i, 2) * jerk_0);
        z = 2.0 * traj_coeffs(i, 3);

        ddx.row(i) = (x + y + w).array() + z;
      }
      return ddx;
    }
    static VEC3 get_max_abs_acceleration(FifthOrderPolynomial traj, double tmax)
    {
      Eigen::Matrix<double, 3, 4> acceleration = get_acceleration_at_jerk_zero(traj, tmax);
      return acceleration.cwiseAbs().rowwise().maxCoeff();
    }
    static VEC3 get_max_abs_accelerations_from_time(Eigen::Matrix<double, 6, 3> b_matrix, double T)
    {
      if (T <= 0)
      {
        return VEC3::Constant(INFINITY, INFINITY, INFINITY);
      }
      if (b_matrix.isZero())
      {
        return VEC3::Zero();
      }
      FifthOrderPolynomial traj(b_matrix, T);
      return get_max_abs_acceleration(traj, T);
    }
    static double get_global_max_abs_accelerations_from_time(Eigen::Matrix<double, 6, 3> b_matrix, double T)
    {
      VEC3 max_abs_acceleration = get_max_abs_accelerations_from_time(b_matrix, T);
      return max_abs_acceleration.maxCoeff();
    }

   public:
    OptimalFifthOrderPolynomial(
        VEC3 pos,
        VEC3 vel,
        VEC3 acc,
        VEC3 tgt_pos,
        VEC3 tgt_vel,
        VEC3 tgt_acc,
        double T,
        double tol = 1e-8,
        unsigned int N = 1000,
        bool _solve = true)
        : FifthOrderPolynomial(pos, vel, acc, tgt_pos, tgt_vel, tgt_acc, T, _solve = _solve),
          tol(tol),
          N(N)
    {
    }
    OptimalFifthOrderPolynomial(
        Eigen::Matrix<double, 6, 3> b_matrix,
        double T,
        double tol = 1e-8,
        unsigned int N = 1000,
        bool _solve = true)
        : FifthOrderPolynomial(b_matrix, T, _solve = _solve),
          tol(tol),
          N(N){};

    OptimalFifthOrderPolynomial solve()
    {
      std::function<double(double)> f = [this](double T) -> double
      { return get_global_max_abs_accelerations_from_time(this->b_matrix, T); };
      double t_optim = solvers::bisection_with_right_expansion(f, 0, this->tmax, this->tol, this->N);
      this->T = t_optim;
      this->A_matrix = calc_A_matrix(this->T);
      FifthOrderPolynomial::solve();
    }
  };
}  // namespace jdrones::polynomial

#endif  // POLYNOMIAL_H
