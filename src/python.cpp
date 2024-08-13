/*
 * Copyright (c) 2024.  Jan-Hendrik Ewers
 * SPDX-License-Identifier: GPL-3.0-only
 */

#include <pybind11/eigen.h>
#include <pybind11/functional.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "jdrones/dynamics.h"
#include "jdrones/controllers.h"
#include "jdrones/data.h"
#include "jdrones/envs.h"
#include "jdrones/polynomial.h"

namespace py = pybind11;
using namespace py::literals;
using namespace jdrones::types;

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

PYBIND11_MODULE(_core, m)
{
  m.doc() = "A C++ library to speed jdrones computations";

  py::class_<Eigen::Matrix<double, 20, 1>>(m, "EigenMatrix20d1");
  py::class_<jdrones::data::State, Eigen::Matrix<double, 20, 1>>(m, "State").def(py::init<>()).def(py::init<const State&>());

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

  py::class_<jdrones::envs::LQRDroneEnv>(m, "LQRDroneEnv")
      .def(py::init<double, State, Eigen::Matrix<double, 4, 12>>())
      .def(py::init<double, State>())
      .def(py::init<double>())
      .def("reset", py::overload_cast<>(&jdrones::envs::LQRDroneEnv::reset))
      .def("reset", py::overload_cast<State>(&jdrones::envs::LQRDroneEnv::reset))
      .def("step", &jdrones::envs::LQRDroneEnv::step)
      .def_property_readonly("env", &jdrones::envs::LQRDroneEnv::get_env)
      .def("set_K", &jdrones::envs::LQRDroneEnv::set_K);

  py::class_<jdrones::controllers::LQRController>(m, "LQRController")
      .def(py::init<Eigen::Matrix<double, 4, 12>>())
      .def("reset", py::overload_cast<>(&jdrones::controllers::LQRController::reset))
      .def("__call__", py::overload_cast<State, State>(&jdrones::controllers::LQRController::operator()))
      .def_property("K", &jdrones::controllers::LQRController::get_K, &jdrones::controllers::LQRController::set_K);
}