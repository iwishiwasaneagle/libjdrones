/*
 * Copyright (c) 2024.  Jan-Hendrik Ewers
 * SPDX-License-Identifier: GPL-3.0-only
 */

#include "jdrones/transforms.h"

namespace jdrones
{
  Eigen::Vector3d quat_to_euler(Eigen::Vector4d quat)
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

    Eigen::Vector3d euler{ roll, pitch, yaw };
    return euler;
  }

  Eigen::Vector4d euler_to_quat(Eigen::Vector3d rpy)
  {
    double roll, pitch, yaw;
    roll = rpy(0);
    pitch = rpy(1);
    yaw = rpy(2);

    float cy, sy, cp, sp, cr, sr, w, x, y, z;

    cy = cos(yaw * 0.5);
    sy = sin(yaw * 0.5);
    cp = cos(pitch * 0.5);
    sp = sin(pitch * 0.5);
    cr = cos(roll * 0.5);
    sr = sin(roll * 0.5);

    w = cr * cp * cy + sr * sp * sy;
    x = sr * cp * cy - cr * sp * sy;
    y = cr * sp * cy + sr * cp * sy;
    z = cr * cp * sy - sr * sp * cy;

    Eigen::Vector4d quat{ x, y, z, w };
    return quat;
  }

  Eigen::Matrix<double, 3, 3> quat_to_rotmat(Eigen::Vector4d quat)
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
  Eigen::Matrix<double, 3, 3> euler_to_rotmat(Eigen::Vector3d rpy)
  {
    return quat_to_rotmat(euler_to_quat(rpy));
  }
}  // namespace jdrones
