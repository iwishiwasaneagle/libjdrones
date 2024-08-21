/*
 * Copyright (c) 2024.  Jan-Hendrik Ewers
 * SPDX-License-Identifier: GPL-3.0-only
 */

//
// Created by jhewers on 19/08/24.
//

#ifndef GYMNASIUM_H
#define GYMNASIUM_H

#include <eigen3/Eigen/Eigen>
#include <map>
#include <tuple>

namespace jdrones::gymnasium
{
  template<class ReturnType, class ResetType, class ActionType>
  class Env
  {
   public:
    virtual std::tuple<ReturnType, std::map<std::string, Eigen::VectorXd>> reset() = 0;
    virtual std::tuple<ReturnType, std::map<std::string, Eigen::VectorXd>> reset(ResetType) = 0;
    virtual std::tuple<ReturnType, double, bool, bool, std::map<std::string, Eigen::VectorXd>> step(ActionType) = 0;
  };
}  // namespace jdrones::gymnasium

#endif  // GYMNASIUM_H
