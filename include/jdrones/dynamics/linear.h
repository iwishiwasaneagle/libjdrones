/*
 * Copyright (c) 2024.  Jan-Hendrik Ewers
 * SPDX-License-Identifier: GPL-3.0-only
 */



#ifndef LINEAR_H
#define LINEAR_H
#include <eigen3/Eigen/Core>
#include "jdrones/data.h"
#include "jdrones/dynamics/base.h"

namespace jdrones::dynamics
{

using namespace jdrones::data;
  class LinearDynamicModelDroneEnv : public BaseDynamicModelDroneEnv
  {
   public:
    using BaseDynamicModelDroneEnv::BaseDynamicModelDroneEnv;

   protected:
    State calc_dstate(VEC4) override;
  };
}
#endif //LINEAR_H
