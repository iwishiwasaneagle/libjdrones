/*
 * Copyright (c) 2024.  Jan-Hendrik Ewers
 * SPDX-License-Identifier: GPL-3.0-only
 */

#ifndef SOLVERS_H
#define SOLVERS_H

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
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

  /**
   * @brief Arimoto Potter method for continuous model of Riccati equation
   * @authors: Horibe Takamasa
   *
   * @param A
   * @param B
   * @param Q
   * @param R
   * @param P
   * @return
   */
  template<int xdim, int udim>
  bool solveRiccatiArimotoPotter(
      const Eigen::Matrix<double, xdim, xdim> &A,
      const Eigen::Matrix<double, xdim, udim> &B,
      const Eigen::Matrix<double, xdim, xdim> &Q,
      const Eigen::Matrix<double, udim, udim> &R,
      Eigen::Matrix<double, xdim, xdim> &P)
  {
    // set Hamilton matrix
    Eigen::MatrixXd Ham = Eigen::MatrixXd::Zero(2 * xdim, 2 * xdim);
    Ham << A, -B * R.inverse() * B.transpose(), -Q, -A.transpose();

    // calc eigenvalues and eigenvectors
    Eigen::EigenSolver<Eigen::MatrixXd> Eigs(Ham);

    // extract stable eigenvectors into 'eigvec'
    Eigen::MatrixXcd eigvec = Eigen::MatrixXcd::Zero(2 * xdim, xdim);
    int j = 0;
    for (int i = 0; i < 2 * xdim; ++i)
    {
      if (Eigs.eigenvalues()[i].real() < 0.)
      {
        eigvec.col(j) = Eigs.eigenvectors().block(0, i, 2 * xdim, 1);
        ++j;
      }
    }

    // calc P with stable eigen vector matrix
    Eigen::MatrixXcd Vs_1, Vs_2;
    Vs_1 = eigvec.block(0, 0, xdim, xdim);
    Vs_2 = eigvec.block(xdim, 0, xdim, xdim);
    P = (Vs_2 * Vs_1.inverse()).real();

    return true;
  }
  /**
   * @brief Interation method for continuous model of Riccati equation
   * @authors: Horibe Takamasa
   *
   * @param A
   * @param B
   * @param Q
   * @param R
   * @param P
   * @param dt
   * @param tolerance
   * @param iter_max
   * @return
   */
  template<int xdim, int udim>
  bool solveRiccatiIterationC(
      const Eigen::Matrix<double, xdim, xdim> &A,
      const Eigen::Matrix<double, xdim, udim> &B,
      const Eigen::Matrix<double, xdim, xdim> &Q,
      const Eigen::Matrix<double, udim, udim> &R,
      Eigen::Matrix<double, xdim, xdim> &P,
      const double dt = 0.001,
      const double &tolerance = 1.E-5,
      const unsigned int iter_max = 100000)
  {
    P = Q;  // initialize

    Eigen::MatrixXd P_next;

    Eigen::MatrixXd AT = A.transpose();
    Eigen::MatrixXd BT = B.transpose();
    Eigen::MatrixXd Rinv = R.inverse();

    double diff;
    for (int i = 0; i < iter_max; ++i)
    {
      P_next = P + (P * A + AT * P - P * B * Rinv * BT * P + Q) * dt;
      diff = fabs((P_next - P).maxCoeff());
      P = P_next;
      if (diff < tolerance)
      {
        return true;
      }
    }
    return false;  // over iteration limit
  }

}  // namespace jdrones::solvers

#endif  // SOLVERS_H
