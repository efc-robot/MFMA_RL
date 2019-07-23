from MF_env import MultiFidelityEnv
from MSE import MSE_backend
from MF_env import basic
from scenario.paser import  parse_senario
import time
import numpy as np
import math
from MF_env import policy


def check_done(state_list):
    all_done = True
    for state in state_list:
        all_done = all_done and not state.movable
    return all_done





n_policy = policy.RVO_policy()

fps = 50.0
dt = 0.02
ctrl_fps = 10.0

scenario = parse_senario('./scenario/scenario.yaml')
back_end = MSE_backend.MSE_backend(scenario,fps,dt)
env = MultiFidelityEnv.MultiFidelityEnv(scenario,back_end)
env.rollout_reset()
env.rollout(n_policy.inference,ctrl_fps,check_done)
trajectoy = env.get_trajectoy()

env.close()