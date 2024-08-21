/*
 * Copyright (c) 2024.  Jan-Hendrik Ewers
 * SPDX-License-Identifier: GPL-3.0-only
 */

#include "jdrones/transforms.h"

namespace jdrones
{
  VEC3 quat_to_euler(VEC4 quat)
  {
    double x, y, z, w;
    x = quat(0);
    y = quat(1);
    z = quat(2);
    w = quat(3);

    double sinr_cosp, cosr_cosp, siny_cosp, cosy_cosp, sinp, cosp;
    double roll, pitch, yaw;

    sinr_cosp = 2 * (w * x + y * z);
    cosr_cosp = 1 - 2 * (x * x + y * y);
    roll = atan2(sinr_cosp, cosr_cosp);
    sinp = sqrt(1 + 2 * (w * y - x * z));
    cosp = sqrt(1 - 2 * (w * y - x * z));
    pitch = 2 * atan2(sinp, cosp) - M_PI / 2;

    siny_cosp = 2 * (w * z + x * y);
    cosy_cosp = 1 - 2 * (y * y + z * z);
    yaw = atan2(siny_cosp, cosy_cosp);

    VEC3 euler{ roll, pitch, yaw };
    return euler;
  }

  VEC4 euler_to_quat(VEC3 rpy)
  {
    const double half_roll = rpy(0) * 0.5;
    const double half_pitch = rpy(1) * 0.5;
    const double half_yaw = rpy(2) * 0.5;

    const double cy = cos(half_yaw);
    const double sy = sin(half_yaw);
    const double cp = cos(half_pitch);
    const double sp = sin(half_pitch);
    const double cr = cos(half_roll);
    const double sr = sin(half_roll);

    const double w = cr * cp * cy + sr * sp * sy;
    const double x = sr * cp * cy - cr * sp * sy;
    const double y = cr * sp * cy + sr * cp * sy;
    const double z = cr * cp * sy - sr * sp * cy;

    return VEC4{ x, y, z, w };
  }

  Eigen::Matrix<double, 3, 3> quat_to_rotmat(VEC4 quat)
  {
    double x, y, z, w;
    x = quat(0);
    y = quat(1);
    z = quat(2);
    w = quat(3);
    double x2, y2, z2;
    x2 = x * x;
    y2 = y * y;
    z2 = z * z;
    double xy, xz, xw, yz, yw, zw;

    xy = x * y;
    xz = x * z;
    xw = x * w;
    yz = y * z;
    yw = y * w;
    zw = z * w;
    Eigen::Matrix<double, 3, 3> rot_mat;
    rot_mat << 1 - 2 * (y2 + z2), 2 * (xy - zw), 2 * (xz + yw), 2 * (xy + zw), 1 - 2 * (x2 + z2), 2 * (yz - xw),
        2 * (xz - yw), 2 * (yz + xw), 1 - 2 * (x2 + y2);
    return rot_mat;
  }
  Eigen::Matrix<double, 3, 3> euler_to_rotmat(VEC3 rpy)
  {
    return quat_to_rotmat(euler_to_quat(rpy));
  }
}  // namespace jdrones
