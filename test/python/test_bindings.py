#  Copyright (c) 2024.  Jan-Hendrik Ewers
#  SPDX-License-Identifier: GPL-3.0-only
import pytest
import numpy as np
from libjdrones import NonLinearDynamicModelDroneEnv, LinearDynamicModelDroneEnv, State

@pytest.mark.parametrize("env_cls", [NonLinearDynamicModelDroneEnv,LinearDynamicModelDroneEnv])
def test_dynamic_model_envs(env_cls):
    env = env_cls(0.1)
    u = np.ones(4)
    obs, rew, term, trunc, info = env.step(u)
    assert not np.isnan(np.sum(obs))
