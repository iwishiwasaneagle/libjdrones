/*
 * Copyright (c) 2024.  Jan-Hendrik Ewers
 * SPDX-License-Identifier: GPL-3.0-only
 */

#ifndef TRANSFORMS_H
#define TRANSFORMS_H
#include <eigen3/Eigen/Core>

namespace jdrones
{
  Eigen::Vector3d quat_to_euler(Eigen::Vector4d quat);
  Eigen::Vector4d euler_to_quat(Eigen::Vector3d rpy);
  Eigen::Matrix<double, 3, 3> quat_to_rotmat(Eigen::Vector4d quat);
  Eigen::Matrix<double, 3, 3> euler_to_rotmat(Eigen::Vector3d rpy);
}  // namespace jdrones

#endif  // TRANSFORMS_H
