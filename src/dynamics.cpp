/*
 * Copyright (c) 2024.  Jan-Hendrik Ewers
 * SPDX-License-Identifier: GPL-3.0-only
 */
#include "jdrones/dynamics.h"

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

VEC4 jdrones::dynamics::BaseDynamicModelDroneEnv::rpm2rpyT(VEC4 rpm)
{
  return VEC4(mixing_matrix * rpm);
}
VEC4 jdrones::dynamics::BaseDynamicModelDroneEnv::rpyT2rpm(VEC4 rpm)
{
  return VEC4(mixing_matrix.inverse() * rpm);
}

State jdrones::dynamics::BaseDynamicModelDroneEnv::step(VEC4 rpm)
{
  State dstate = this->calc_dstate(rpm);
  this->state += dt * dstate;
  this->state.set_prop_omega(rpm);
  this->state.set_quat(euler_to_quat(this->state.get_rpy()));
  return this->state;
}

State jdrones::dynamics::NonlinearDynamicModelDroneEnv::calc_dstate(VEC4 rpm)
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

State jdrones::dynamics::LinearDynamicModelDroneEnv::calc_dstate(VEC4 rpm)
{
  X x = state_to_x(this->state);
  Eigen::Vector4d rpyT = rpm2rpyT(rpm);
  X dx = (this->A * x) + (this->B * rpyT) + (this->C);
  return x_to_state(dx);
}
