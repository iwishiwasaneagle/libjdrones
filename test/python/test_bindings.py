#  Copyright (c) 2024-2025.  Jan-Hendrik Ewers
#  SPDX-License-Identifier: GPL-3.0-only
import pytest
import numpy as np
from libjdrones import LQRDroneEnv
from libjdrones import NonLinearDynamicModelDroneEnv, LinearDynamicModelDroneEnv, State

@pytest.mark.parametrize("env_cls", [NonLinearDynamicModelDroneEnv,LinearDynamicModelDroneEnv])
def test_dynamic_model_envs(env_cls):
    env = env_cls(0.1)
    u = np.ones(4)
    obs, rew, term, trunc, info = env.step(u)
    assert not np.isnan(np.sum(obs))

def test_lqr_drone_env_q_r():
    Q = np.eye(12, dtype=np.float64)
    R = np.eye(4, dtype=np.float64)
    dt = 0.02
    initial_state = State()

    env = LQRDroneEnv(dt, initial_state, Q, R)
    obs, *_ = env.step(initial_state)

    assert np.allclose(obs, initial_state)
