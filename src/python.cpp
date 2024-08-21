/*
 * Copyright (c) 2024.  Jan-Hendrik Ewers
 * SPDX-License-Identifier: GPL-3.0-only
 */

#include <pybind11/eigen.h>
#include <pybind11/functional.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "jdrones/controllers.h"
#include "jdrones/data.h"
#include "jdrones/dynamics/dynamics.h"
#include "jdrones/envs/envs.h"
#include "jdrones/polynomial.h"

namespace py = pybind11;
using namespace py::literals;
using namespace jdrones::data;

class PyBaseDynamicModelDroneEnv : public jdrones::dynamics::BaseDynamicModelDroneEnv
{
 public:
  using jdrones::dynamics::BaseDynamicModelDroneEnv::BaseDynamicModelDroneEnv;

  State calc_dstate(Eigen::Vector4d rpm) override
  {
    PYBIND11_OVERRIDE_PURE(State, BaseDynamicModelDroneEnv, calc_dstate, rpm);
  };
};
class PyBasePolynomial : public jdrones::polynomial::BasePolynomial
{
 public:
  using jdrones::polynomial::BasePolynomial::BasePolynomial;

  VEC3 position(double t) override
  {
    PYBIND11_OVERRIDE_PURE(VEC3, BasePolynomial, position, t);
  }
  VEC3 velocity(double t) override
  {
    PYBIND11_OVERRIDE_PURE(VEC3, BasePolynomial, velocity, t);
  }
  VEC3 acceleration(double t) override
  {
    PYBIND11_OVERRIDE_PURE(VEC3, BasePolynomial, acceleration, t);
  }
  void solve() override
  {
    PYBIND11_OVERRIDE_PURE(void, BasePolynomial, solve);
  }
  Eigen::Matrix<double, -1, 3> get_coeffs() override
  {
    using M = Eigen::Matrix<double, -1, 3>;
    PYBIND11_OVERRIDE_PURE(M, BasePolynomial, get_coeffs);
  }
};
template<class Polynomial>
class PyBasePolynomialPositionDroneEnv : public jdrones::envs::BasePolynomialPositionDroneEnv<Polynomial>
{
 public:
  using jdrones::envs::BasePolynomialPositionDroneEnv<Polynomial>::BasePolynomialPositionDroneEnv;

  Polynomial calc_traj(VEC3 pos, VEC3 vel, VEC3 tgt_pos, VEC3 tgt_vel, std::map<std::string, double> params) override
  {
    PYBIND11_OVERRIDE_PURE(
        Polynomial,
        jdrones::envs::BasePolynomialPositionDroneEnv<Polynomial>,
        calc_traj,
        pos,
        vel,
        tgt_pos,
        tgt_vel,
        params);
  }
  std::map<std::string, double> get_traj_params() override
  {
    using PARAMS = std::map<std::string, double>;
    PYBIND11_OVERRIDE_PURE(PARAMS, jdrones::envs::BasePolynomialPositionDroneEnv<Polynomial>, get_traj_params);
  }
};
template<class Polynomial>
void register_base_polynomial_position_drone_env(py::module& m, std::string typestr)
{
  using BPPDE = jdrones::envs::BasePolynomialPositionDroneEnv<Polynomial>;
  py::class_<BPPDE, PyBasePolynomialPositionDroneEnv<Polynomial>>(
      m, (std::string("BasePolynomialPositionDroneEnv") + typestr).c_str())
      .def(py::init<double, State>())
      .def(py::init<double>())
      .def("calc_traj", &BPPDE::calc_traj)
      .def("get_traj_params", &BPPDE::get_traj_params)
      .def("reset", py::overload_cast<>(&BPPDE::reset))
      .def("reset", py::overload_cast<State>(&BPPDE::reset))
      .def("step", py::overload_cast<VEC3>(&BPPDE::step))
      .def("step", py::overload_cast<std::pair<VEC3, VEC3>>(&BPPDE::step))
      .def_property_readonly("env", &BPPDE::get_env)
      .def_property_readonly("dt", &BPPDE::get_dt);
}

template<class ReturnType, class ResetType, class ActionType>
class PyEnv : public jdrones::gymnasium::Env<ReturnType, ResetType, ActionType>
{
 public:
  std::tuple<ReturnType, std::map<std::string, Eigen::VectorXd>> reset() override
  {
    using PARAMS = std::tuple<ReturnType, std::map<std::string, Eigen::VectorXd>>;
    using ENV = jdrones::gymnasium::Env<ReturnType, ResetType, ActionType>;
    PYBIND11_OVERRIDE_PURE(PARAMS, ENV, reset);
  }
  std::tuple<ReturnType, std::map<std::string, Eigen::VectorXd>> reset(ResetType option) override
  {
    using PARAMS = std::tuple<ReturnType, std::map<std::string, Eigen::VectorXd>>;
    using ENV = jdrones::gymnasium::Env<ReturnType, ResetType, ActionType>;
    PYBIND11_OVERRIDE_PURE(PARAMS, ENV, reset, option);
  }
  std::tuple<ReturnType, double, bool, bool, std::map<std::string, Eigen::VectorXd>> step(ActionType action) override
  {
    using PARAMS = std::tuple<ReturnType, double, bool, bool, std::map<std::string, Eigen::VectorXd>>;
    using ENV = jdrones::gymnasium::Env<ReturnType, ResetType, ActionType>;
    PYBIND11_OVERRIDE_PURE(PARAMS, ENV, step, action);
  }
};

template<class ReturnType, class ResetType, class ActionType>
void register_gymnasium_env(py::module& m, std::string tag)
{
  using ENV = jdrones::gymnasium::Env<ReturnType, ResetType, ActionType>;
  using PYENV = PyEnv<ReturnType, ResetType, ActionType>;
  std::string class_name = "Env";
  class_name += "_" + tag;
  py::class_<ENV, PYENV>(m, class_name.c_str())
      .def(py::init<>())
      .def("reset", py::overload_cast<>(&ENV::reset))
      .def("reset", py::overload_cast<ResetType>(&ENV::reset))
      .def("step", &ENV::step);
}

template<int xdim, int udim>
void register_lqr_controller(py::module& m)
{
  using LQR = jdrones::controllers::LQRController<xdim, udim>;
  std::string class_name = "LQRController";
  class_name += "_" + std::to_string(xdim) + "_" + std::to_string(udim);
  py::class_<LQR>(m, class_name.c_str())
      .def(py::init<
           Eigen::Matrix<double, xdim, xdim>,
           Eigen::Matrix<double, xdim, udim>,
           Eigen::Matrix<double, xdim, xdim>,
           Eigen::Matrix<double, udim, udim>>())
      .def(py::init<Eigen::Matrix<double, udim, xdim>>())
      .def("reset", py::overload_cast<>(&LQR::reset))
      .def("__call__", py::overload_cast<Eigen::Matrix<double, xdim, 1>, Eigen::Matrix<double, xdim, 1>>(&LQR::operator()))
      .def_property_readonly("K", &LQR::get_K);
}

PYBIND11_MODULE(_core, m)
{
  m.doc() = "A C++ library to speed jdrones computations";

  register_gymnasium_env<State, State, State>(m, "state_state_state");
  register_gymnasium_env<State, State, VEC4>(m, "state_state_vec4");

  py::class_<Eigen::Matrix<double, 20, 1>>(m, "EigenMatrix20d1");
  py::class_<jdrones::data::State, Eigen::Matrix<double, 20, 1>>(m, "State")
      .def(py::init<>())
      .def(py::init<const State&>())
      .def_property("pos", &State::get_pos, &State::set_pos)
      .def_property("quat", &State::get_quat, &State::set_quat)
      .def_property("vel", &State::get_vel, &State::set_vel)
      .def_property("ang_vel", &State::get_ang_vel, &State::get_ang_vel)
      .def_property("rpy", &State::get_rpy, &State::set_rpy)
      .def_property("prop_omega", &State::get_prop_omega, &State::get_prop_omega);

  py::class_<jdrones::dynamics::BaseDynamicModelDroneEnv, PyBaseDynamicModelDroneEnv>(m, "BaseDynamicModelDroneEnv")
      .def(py::init<double>())
      .def(py::init<double, State>())
      .def_property_readonly("dt", &jdrones::dynamics::BaseDynamicModelDroneEnv::get_dt)
      .def_property_readonly("A", &jdrones::dynamics::BaseDynamicModelDroneEnv::get_A)
      .def_property_readonly("B", &jdrones::dynamics::BaseDynamicModelDroneEnv::get_B)
      .def_property_readonly("C", &jdrones::dynamics::BaseDynamicModelDroneEnv::get_C)
      .def_property_readonly("state", &jdrones::dynamics::BaseDynamicModelDroneEnv::get_state)
      .def("reset", py::overload_cast<>(&jdrones::dynamics::BaseDynamicModelDroneEnv::reset))
      .def("reset", py::overload_cast<State>(&jdrones::dynamics::BaseDynamicModelDroneEnv::reset))
      .def("step", &jdrones::dynamics::BaseDynamicModelDroneEnv::step);

  py::class_<jdrones::dynamics::LinearDynamicModelDroneEnv, jdrones::dynamics::BaseDynamicModelDroneEnv>(
      m, "LinearDynamicModelDroneEnv")
      .def(py::init<double>())
      .def(py::init<double, State>());

  py::class_<jdrones::dynamics::NonlinearDynamicModelDroneEnv, jdrones::dynamics::BaseDynamicModelDroneEnv>(
      m, "NonLinearDynamicModelDroneEnv")
      .def(py::init<double>())
      .def(py::init<double, State>());

  register_lqr_controller<12, 4>(m);
  py::class_<jdrones::envs::LQRDroneEnv, jdrones::gymnasium::Env<State, State, State>>(m, "LQRDroneEnv")
      .def(py::init<double, State>())
      .def(py::init<double>())
      .def("reset", py::overload_cast<>(&jdrones::envs::LQRDroneEnv::reset))
      .def("reset", py::overload_cast<State>(&jdrones::envs::LQRDroneEnv::reset))
      .def("step", &jdrones::envs::LQRDroneEnv::step)
      .def_property_readonly("env", &jdrones::envs::LQRDroneEnv::get_env);

  py::class_<jdrones::polynomial::BasePolynomial, PyBasePolynomial>(m, "BasePolynomial")
      .def(py::init<VEC3, VEC3, VEC3, VEC3, VEC3, VEC3, double>())
      .def("position", &jdrones::polynomial::BasePolynomial::position)
      .def("velocity", &jdrones::polynomial::BasePolynomial::velocity)
      .def("acceleration", &jdrones::polynomial::BasePolynomial::acceleration)
      .def("get_coeffs", &jdrones::polynomial::BasePolynomial::get_coeffs)
      .def("get_T", &jdrones::polynomial::BasePolynomial::get_T)
      .def("solve", &jdrones::polynomial::BasePolynomial::solve);

  py::class_<jdrones::polynomial::FifthOrderPolynomial, jdrones::polynomial::BasePolynomial>(m, "FifthOrderPolynomial")
      .def(py::init<VEC3, VEC3, VEC3, VEC3, VEC3, VEC3, double>())
      .def("jerk", py::overload_cast<double>(&jdrones::polynomial::FifthOrderPolynomial::jerk))
      .def("snap", py::overload_cast<double>(&jdrones::polynomial::FifthOrderPolynomial::snap))
      .def("solve", &jdrones::polynomial::FifthOrderPolynomial::solve);

  py::class_<jdrones::polynomial::OptimalFifthOrderPolynomial, jdrones::polynomial::FifthOrderPolynomial>(
      m, "OptimalFifthOrderPolynomial")
      .def(py::init<VEC3, VEC3, VEC3, VEC3, VEC3, VEC3, double, double, double, unsigned int>())
      .def("solve", &jdrones::polynomial::OptimalFifthOrderPolynomial::solve);

  register_base_polynomial_position_drone_env<jdrones::polynomial::FifthOrderPolynomial>(m, "5O");
  py::class_<
      jdrones::envs::FifthOrderPolyPositionDroneEnv,
      jdrones::envs::BasePolynomialPositionDroneEnv<jdrones::polynomial::FifthOrderPolynomial>>(
      m, "FifthOrderPolyPositionDroneEnv")
      .def(py::init<double, State>())
      .def(py::init<double>())
      .def_property(
          "max_vel",
          &jdrones::envs::FifthOrderPolyPositionDroneEnv::get_max_vel,
          &jdrones::envs::FifthOrderPolyPositionDroneEnv::get_max_vel);
  register_base_polynomial_position_drone_env<jdrones::polynomial::OptimalFifthOrderPolynomial>(m, "Opt5O");
  py::class_<
      jdrones::envs::OptimalFifthOrderPolyPositionDroneEnv,
      jdrones::envs::BasePolynomialPositionDroneEnv<jdrones::polynomial::OptimalFifthOrderPolynomial>>(
      m, "OptimalFifthOrderPolyPositionDroneEnv")
      .def(py::init<double, State, double>())
      .def(py::init<double, State>())
      .def(py::init<double>())
      .def_property(
          "max_acc",
          &jdrones::envs::OptimalFifthOrderPolyPositionDroneEnv::get_max_acc,
          &jdrones::envs::OptimalFifthOrderPolyPositionDroneEnv::set_max_acc);
}