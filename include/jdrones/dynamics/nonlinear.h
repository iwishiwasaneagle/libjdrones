/*
 * Copyright (c) 2024.  Jan-Hendrik Ewers
 * SPDX-License-Identifier: GPL-3.0-only
 */

#ifndef NONLINEAR_H
#define NONLINEAR_H
#include <eigen3/Eigen/Core>
#include "jdrones/data.h"
#include "jdrones/dynamics/base.h"

using namespace jdrones::data;
namespace jdrones::dynamics
{
  class NonlinearDynamicModelDroneEnv : public BaseDynamicModelDroneEnv
  {
   public:
    using BaseDynamicModelDroneEnv::BaseDynamicModelDroneEnv;

   protected:
    State calc_dstate(VEC4) override;
  };

}  // namespace jdrones::dynamics
#endif  // NONLINEAR_H
