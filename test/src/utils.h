/*
 * Copyright (c) 2024.  Jan-Hendrik Ewers
 * SPDX-License-Identifier: GPL-3.0-only
 */

//
// Created by jhewers on 19/08/24.
//

#ifndef UTILS_H
#define UTILS_H

#include <cmath>

static bool almost_equal_tol(const double& lhs, const double& rhs, const double& tol)
{
  return std::abs(lhs - rhs) < tol;
}

static bool almost_equal(const double& lhs, const double& rhs)
{
  return almost_equal_tol(lhs, rhs, 1e-9);
}

#endif  // UTILS_H
