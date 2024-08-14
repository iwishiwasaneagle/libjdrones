/*
 * Copyright (c) 2024.  Jan-Hendrik Ewers
 * SPDX-License-Identifier: GPL-3.0-only
 */

/*
 * After compilation in pycharm using various cmake flags, the following results are true for v0.1.0-1-g9c201a9
 *
 * $ fd "benchmark_lqrdroneenv" -t x -HI -x bash -c "echo {} && time {}"
 * ./cmake-build-release/bench/benchmark_lqrdroneenv.cpp
 *
 * real    0m0.011s
 * user    0m0.011s
 * sys     0m0.000s
 * ./cmake-build-release-vectorization/bench/benchmark_lqrdroneenv.cpp
 *
 * real    0m0.053s
 * user    0m0.053s
 * sys     0m0.000s
 * ./cmake-build-debug/bench/benchmark_lqrdroneenv.cpp
 *
 * real    0m1.842s
 * user    0m1.828s
 * sys     0m0.003s
 * ./cmake-build-debug-coverage/bench/benchmark_lqrdroneenv.cpp
 *
 * real    0m2.179s
 * user    0m2.178s
 * sys     0m0.001s
 * ./cmake-build-debug-vectorization/bench/benchmark_lqrdroneenv.cpp
 *
 * real    0m2.701s
 * user    0m2.701s
 * sys     0m0.000s
 */

#include "jdrones/envs.h"

int main(){
  jdrones::envs::LQRDroneEnv env(0.01);

  State setpoint;
  setpoint.set_pos({10,10,10});

  for(int i=0;i<100000;i++)
  {
    env.step(setpoint);
  }
}

