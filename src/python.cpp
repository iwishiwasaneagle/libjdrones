/*
 * Copyright (c) 2024.  Jan-Hendrik Ewers
 * SPDX-License-Identifier: GPL-3.0-only
 */

#include <pybind11/eigen.h>
#include <pybind11/functional.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "jdrones/envs.h"

namespace py = pybind11;
using namespace py::literals;

class PyBaseDynamicModelDroneEnv : public jdrones::envs::BaseDynamicModelDroneEnv
{
 public:
  using jdrones::envs::BaseDynamicModelDroneEnv::BaseDynamicModelDroneEnv;

  State calc_dstate(Eigen::Vector4d rpm) override
  {
    PYBIND11_OVERRIDE_PURE(State, BaseDynamicModelDroneEnv, calc_dstate, rpm);
  };
};

PYBIND11_MODULE(_core, m)
{
  m.doc() = "A C++ library to speed jdrones computations";

  py::class_<Eigen::Matrix<double, 20, 1>>(m, "EigenMatrix20d1");
  py::class_<jdrones::data::State, Eigen::Matrix<double, 20, 1>>(m, "State").def(py::init<>()).def(py::init<const State&>());

  py::class_<jdrones::envs::BaseDynamicModelDroneEnv, PyBaseDynamicModelDroneEnv>(m, "BaseDynamicModelDroneEnv")
      .def(py::init<float>())
      .def(py::init<float, State>())
      .def_property_readonly("dt", &jdrones::envs::BaseDynamicModelDroneEnv::get_dt)
      .def_property_readonly("A", &jdrones::envs::BaseDynamicModelDroneEnv::get_A)
      .def_property_readonly("B", &jdrones::envs::BaseDynamicModelDroneEnv::get_B)
      .def_property_readonly("C", &jdrones::envs::BaseDynamicModelDroneEnv::get_C)
      .def("reset", py::overload_cast<>(&jdrones::envs::BaseDynamicModelDroneEnv::reset))
      .def("reset", py::overload_cast<State>(&jdrones::envs::BaseDynamicModelDroneEnv::reset))
      .def("step",&jdrones::envs::BaseDynamicModelDroneEnv::step);

  py::class_<jdrones::envs::LinearDynamicModelDroneEnv, jdrones::envs::BaseDynamicModelDroneEnv>(m, "LinearDynamicModelDroneEnv")
      .def(py::init<float>())
      .def(py::init<float, State>());

  py::class_<jdrones::envs::NonlinearDynamicModelDroneEnv, jdrones::envs::BaseDynamicModelDroneEnv>(
      m, "NonLinearDynamicModelDroneEnv")
      .def(py::init<float>())
      .def(py::init<float, State>());
}