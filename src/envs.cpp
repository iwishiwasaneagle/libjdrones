/*
 * Copyright (c) 2024.  Jan-Hendrik Ewers
 * SPDX-License-Identifier: GPL-3.0-only
 */
#include "jdrones/envs.h"

#include "jdrones/transforms.h"

Eigen::Vector4d jdrones::envs::BaseDynamicModelDroneEnv::rpm2rpyT(Eigen::Vector4d rpm)
{
  return Eigen::Vector4d(mixing_matrix * rpm);
}

State jdrones::envs::BaseDynamicModelDroneEnv::step(Eigen::Vector4d rpm)
{
  State dstate = this->calc_dstate(rpm);
  this->state += dt * dstate;
  this->state.set_prop_omega(rpm);
  this->state.set_quat(euler_to_quat(this->state.get_rpy()));
  return this->state;
}

State jdrones::envs::NonlinearDynamicModelDroneEnv::calc_dstate(Eigen::Vector4d rpm)
{
  Eigen::Vector4d rpm_squared = rpm.array().pow(2);
  auto u_star = rpm2rpyT(rpm_squared);
  Eigen::Vector3d UZ{ 0, 0, 1 };
  Eigen::Vector3d rpy(this->state.block<3, 1>(7, 0).transpose());
  Eigen::Vector3d ang_vel(this->state.block<3, 1>(13, 0).transpose());
  Eigen::Vector3d vel(this->state.block<3, 1>(10, 0).transpose());
  Eigen::Matrix<double, 3, 3> R_W_Q = euler_to_rotmat(rpy);
  Eigen::Matrix<double, 3, 3> R_Q_W = R_W_Q.transpose();
  Eigen::Matrix<double, 3, 3> inv_inerties = I.asDiagonal().inverse();

  Eigen::Vector3d body_vel = R_W_Q * vel;
  Eigen::Vector3d body_vel_squared = body_vel.array().pow(2);
  Eigen::Vector3d body_drag_mag = drag_coeffs.cwiseProduct(body_vel_squared);
  Eigen::Vector3d drag_force = (-1 * vel.cwiseSign()).cwiseProduct(R_Q_W * body_drag_mag);

  State dstate;
  dstate.setZero();
  dstate.set_pos(vel);
  dstate.set_rpy(ang_vel);
  dstate.set_vel(-mass * jdrones::constants::g * UZ + (R_W_Q * UZ) * u_star(3) + drag_force);
  dstate.set_ang_vel(inv_inerties * u_star.block<3, 1>(0, 0));
  return dstate;
}

State jdrones::envs::LinearDynamicModelDroneEnv::calc_dstate(Eigen::Vector4d rpm)
{
  X x = state_to_x(this->state);
  Eigen::Vector4d rpyT = rpm2rpyT(rpm);
  X dx = static_cast<X>((this->A * x) + (this->B * rpyT) + (this->C));
  return x_to_state(dx);
}
