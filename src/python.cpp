/*
 * Copyright (c) 2024.  Jan-Hendrik Ewers
 * SPDX-License-Identifier: GPL-3.0-only
 */

#include <pybind11/eigen.h>
#include <pybind11/functional.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "jdrones/envs.h"
#include "jdrones/polynomial.h"

namespace py = pybind11;
using namespace py::literals;
using namespace jdrones::types;

class PyBaseDynamicModelDroneEnv : public jdrones::envs::BaseDynamicModelDroneEnv
{
 public:
  using jdrones::envs::BaseDynamicModelDroneEnv::BaseDynamicModelDroneEnv;

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

  py::class_<jdrones::envs::BaseDynamicModelDroneEnv, PyBaseDynamicModelDroneEnv>(m, "BaseDynamicModelDroneEnv")
      .def(py::init<double>())
      .def(py::init<double, State>())
      .def_property_readonly("dt", &jdrones::envs::BaseDynamicModelDroneEnv::get_dt)
      .def_property_readonly("A", &jdrones::envs::BaseDynamicModelDroneEnv::get_A)
      .def_property_readonly("B", &jdrones::envs::BaseDynamicModelDroneEnv::get_B)
      .def_property_readonly("C", &jdrones::envs::BaseDynamicModelDroneEnv::get_C)
      .def_property_readonly("state", &jdrones::envs::BaseDynamicModelDroneEnv::get_state)
      .def("reset", py::overload_cast<>(&jdrones::envs::BaseDynamicModelDroneEnv::reset))
      .def("reset", py::overload_cast<State>(&jdrones::envs::BaseDynamicModelDroneEnv::reset))
      .def("step", &jdrones::envs::BaseDynamicModelDroneEnv::step);

  py::class_<jdrones::envs::LinearDynamicModelDroneEnv, jdrones::envs::BaseDynamicModelDroneEnv>(
      m, "LinearDynamicModelDroneEnv")
      .def(py::init<double>())
      .def(py::init<double, State>());

  py::class_<jdrones::envs::NonlinearDynamicModelDroneEnv, jdrones::envs::BaseDynamicModelDroneEnv>(
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
      .def(py::init<VEC3, VEC3, VEC3, VEC3, VEC3, VEC3, double, double,double, unsigned int>())
      .def("solve", &jdrones::polynomial::OptimalFifthOrderPolynomial::solve);
}