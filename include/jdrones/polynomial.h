/*
 * Copyright (c) 2024.  Jan-Hendrik Ewers
 * SPDX-License-Identifier: GPL-3.0-only
 */

#ifndef POLYNOMIAL_H
#define POLYNOMIAL_H
#include <eigen3/Eigen/Eigen>

#include "jdrones/data.h"
#include "jdrones/solvers.h"

namespace jdrones::polynomial
{
  using namespace jdrones::data;

  using BMATRIX = Eigen::Matrix<double, 6, 3>;
  using AMATRIX = Eigen::Matrix<double, 6, 6>;
  using COEFFVEC = Eigen::Matrix<double, 6, 1>;
  using COEFFMAT3 = Eigen::Matrix<double, 6, 3>;

  /*****************
   * BasePolynomial *
   *****************/
  class BasePolynomial
  {
   protected:
    BMATRIX b_matrix;
    double T;

    static BMATRIX calc_b_matrix(VEC3 pos, VEC3 vel, VEC3 acc, VEC3 tgt_pos, VEC3 tgt_vel, VEC3 tgt_acc);

   public:
    BasePolynomial(BMATRIX b_matrix, double T) : b_matrix(b_matrix), T(T)
    {
    }
    BasePolynomial(VEC3 pos, VEC3 vel, VEC3 acc, VEC3 tgt_pos, VEC3 tgt_vel, VEC3 tgt_acc, double T)
        : T(T),
          b_matrix(calc_b_matrix(pos, vel, acc, tgt_pos, tgt_vel, tgt_acc))
    {
    }

    virtual VEC3 position(double t) = 0;
    virtual VEC3 velocity(double t) = 0;
    virtual VEC3 acceleration(double t) = 0;

    virtual void solve() = 0;

    virtual Eigen::Matrix<double, -1, 3> get_coeffs() = 0;
    [[nodiscard]] double get_T() const
    {
      return T;
    }
    [[nodiscard]] BMATRIX get_b_matrix() const
    {
      return b_matrix;
    }
  };

  /***********************
   * FifthOrderPolynomial *
   ***********************/
  class FifthOrderPolynomial : public BasePolynomial
  {
   protected:
    AMATRIX A_matrix;
    COEFFMAT3 coeffs;
    COEFFMAT3 vel_coeffs, acc_coeffs, jerk_coeffs, snap_coeffs;

    static AMATRIX calc_A_matrix(double T);

   public:
    FifthOrderPolynomial(BMATRIX b_matrix, double T) : BasePolynomial(b_matrix, T){};

    FifthOrderPolynomial(VEC3 pos, VEC3 vel, VEC3 acc, VEC3 tgt_pos, VEC3 tgt_vel, VEC3 tgt_acc, double T = 10.0)
        : BasePolynomial(pos, vel, acc, tgt_pos, tgt_vel, tgt_acc, T)
    {
    }
    VEC3 position(double t) override;
    VEC3 velocity(double t) override;
    VEC3 acceleration(double t) override;
    VEC3 jerk(double t);
    VEC3 snap(double t);

    static COEFFVEC calc_snap_coeffs(COEFFMAT3 traj, unsigned int i);
    static COEFFMAT3 calc_snap_coeffs(COEFFMAT3 traj);
    static COEFFVEC calc_jerk_coeffs(COEFFMAT3 traj, unsigned int i);
    static COEFFMAT3 calc_jerk_coeffs(COEFFMAT3 traj);
    static COEFFVEC calc_acceleration_coeffs(COEFFMAT3 traj, unsigned int i);
    static COEFFMAT3 calc_acceleration_coeffs(COEFFMAT3 traj);
    static COEFFVEC calc_velocity_coeffs(COEFFMAT3 traj, unsigned int i);
    static COEFFMAT3 calc_velocity_coeffs(COEFFMAT3 traj);

    void solve() override;

    [[nodiscard]] AMATRIX get_a_matrix() const
    {
      return A_matrix;
    }
    [[nodiscard]] Eigen::Matrix<double, -1, 3> get_coeffs() override
    {
      return coeffs;
    }
  };

  /******************************
   * OptimalFifthOrderPolynomial *
   ******************************/
  class OptimalFifthOrderPolynomial : public FifthOrderPolynomial
  {
   protected:
    float max_acc;
    float tol;
    unsigned int N;

   public:
    static VEC4 get_acceleration_at_jerk_0_single_dim(FifthOrderPolynomial *traj, unsigned int i);
    static Eigen::Matrix<double, 4, 3> get_acceleration_at_jerk_zero(FifthOrderPolynomial *traj);
    static VEC3 get_max_abs_acceleration(FifthOrderPolynomial *traj);
    static VEC3 get_max_abs_accelerations_from_time(BMATRIX b, double t);
    static double get_global_max_abs_accelerations_from_time(BMATRIX b, double t);

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

    [[nodiscard]] float get_max_acc() const
    {
      return max_acc;
    }
    [[nodiscard]] float get_tol() const
    {
      return tol;
    }
    [[nodiscard]] unsigned int get_N() const
    {
      return N;
    }
  };
}  // namespace jdrones::polynomial

#endif  // POLYNOMIAL_H
