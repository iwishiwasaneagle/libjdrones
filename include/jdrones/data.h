/*
 * Copyright (c) 2024.  Jan-Hendrik Ewers
 * SPDX-License-Identifier: GPL-3.0-only
 */

#ifndef STATE_H
#define STATE_H
#include <eigen3/Eigen/Eigen>

namespace jdrones::data
{
  typedef Eigen::Matrix<double, 4, 1> VEC4;
  typedef Eigen::Matrix<double, 3, 1> VEC3;
  typedef Eigen::Matrix<double, 2, 1> VEC2;

  class State : public Eigen::Matrix<double, 20, 1>
  {
   public:
    using Eigen::Matrix<double, 20, 1>::Matrix;
    VEC3 get_pos()
    {
      return this->block<3, 1>(0, 0);
    };
    void set_pos(VEC3 pos)
    {
      this->block<3, 1>(0, 0) = pos;
    }
    VEC4 get_quat()
    {
      return this->block<4, 1>(3, 0);
    }
    void set_quat(VEC4 quat)
    {
      this->block<4, 1>(3, 0) = quat;
    }
    VEC3 get_rpy()
    {
      return this->block<3, 1>(7, 0);
    }
    void set_rpy(VEC3 rpy)
    {
      this->block<3, 1>(7, 0) = rpy;
    }
    VEC3 get_vel()
    {
      return this->block<3, 1>(10, 0);
    }
    void set_vel(VEC3 vel)
    {
      this->block<3, 1>(10, 0) = vel;
    }
    VEC3 get_ang_vel()
    {
      return this->block<3, 1>(13, 0);
    }
    void set_ang_vel(VEC3 ang_vel)
    {
      this->block<3, 1>(13, 0) = ang_vel;
    }
    VEC4 get_prop_omega()
    {
      return this->block<4, 1>(16, 0);
    }
    void set_prop_omega(VEC4 prop_omega)
    {
      this->block<4, 1>(16, 0) = prop_omega;
    }

    // This constructor allows you to construct State from Eigen expressions
    template<typename OtherDerived>
    State(const Eigen::MatrixBase<OtherDerived>& other) : Eigen::Matrix<double, 20, 1>(other)
    {
    }

    // This method allows you to assign Eigen expressions to State
    template<typename OtherDerived>
    State& operator=(const Eigen::MatrixBase<OtherDerived>& other)
    {
      this->Eigen::Matrix<double, 20, 1>::operator=(other);
      return *this;
    }

    explicit operator std::vector<State>()
    {
      std::vector<State> states{ *this };
      return states;
    }
  };

  using States = std::vector<State>;

  class X : public Eigen::Matrix<double, 12, 1>
  {
   public:
    using Eigen::Matrix<double, 12, 1>::Matrix;
    VEC3 get_pos()
    {
      return this->block<3, 1>(0, 0);
    };
    void set_pos(VEC3 pos)
    {
      this->block<3, 1>(0, 0) = pos;
    }
    VEC3 get_vel()
    {
      return this->block<3, 1>(3, 0);
    }
    void set_vel(VEC3 vel)
    {
      this->block<3, 1>(3, 0) = vel;
    }
    VEC3 get_rpy()
    {
      return this->block<3, 1>(6, 0);
    }
    void set_rpy(VEC3 vel)
    {
      this->block<3, 1>(6, 0) = vel;
    }
    VEC3 get_ang_vel()
    {
      return this->block<3, 1>(9, 0);
    }
    void set_ang_vel(VEC3 ang_vel)
    {
      this->block<3, 1>(9, 0) = ang_vel;
    }
    // This constructor allows you to construct State from Eigen expressions
    template<typename OtherDerived>
    X(const Eigen::MatrixBase<OtherDerived>& other) : Eigen::Matrix<double, 12, 1>(other)
    {
    }

    // This method allows you to assign Eigen expressions to State
    template<typename OtherDerived>
    X& operator=(const Eigen::MatrixBase<OtherDerived>& other)
    {
      this->Eigen::Matrix<double, 12, 1>::operator=(other);
      return *this;
    }
  };

  constexpr int STATE_TO_X_INDEX[12] = { 0, 1, 2, 10, 11, 12, 7, 8, 9, 13, 14, 15 };

  inline State x_to_state(X x)
  {
    const Eigen::Vector<int, 12> indices(STATE_TO_X_INDEX);
    State state = State::Zero();
    state(indices) = x;
    return state;
  }

  inline X state_to_x(State state)
  {
    const Eigen::Vector<int, 12> indices(STATE_TO_X_INDEX);
    X x = state(indices);
    return x;
  }
}  // namespace jdrones::data

#endif  // STATE_H
