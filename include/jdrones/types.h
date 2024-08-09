/*
 * Copyright (c) 2024.  Jan-Hendrik Ewers
 * SPDX-License-Identifier: GPL-3.0-only
 */

#ifndef TYPES_H
#define TYPES_H

#include <eigen3/Eigen/Core>

namespace jdrones::types
{
  typedef Eigen::Matrix<double, 4, 1> VEC4;
  typedef Eigen::Matrix<double, 3, 1> VEC3;
  typedef Eigen::Matrix<double, 2, 1> VEC2;
}  // namespace jdrones::types

#endif  // TYPES_H
