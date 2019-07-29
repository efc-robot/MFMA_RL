from MF_env import MultiFidelityEnv
from MSE import MSE_backend
from MF_env import basic
from MF_env import policy
import os
from scenario.paser import  parse_senario
from MF_env import policy
import torch

import time
import numpy as np
import math


def if_finish(state_list):
    finish = False
    for state in state_list:
        finish |= state.movable
    return not finish


#fps = 400.0
#dt = 0.025
#ctrl_fpT = 10.0


fps = 40.0
dt = 0.025
ctrl_fpT = 10.0
ctrl_fps = fps*dt*ctrl_fpT

n_policy = policy.RVO_policy()


eval_scenario = parse_senario('./scenario/scenario_eval.yaml')
eval_back_end = MSE_backend.MSE_backend(eval_scenario,fps,dt)
env = MultiFidelityEnv.MultiFidelityEnv(eval_scenario,eval_back_end)



actor = torch.load(os.path.join('./results/MAC/0/314/','actor.pkl'))
nn_policy= policy.NN_policy(actor,0)

env.reset_rollout()
env.rollout(nn_policy.inference,ctrl_fps,if_finish)
traj = env.get_trajectoy()

print([ (trans['reward'],trans['done']) for trans in traj])
#delta_time = np.array([a-b for a,b in zip(env.time_history[1:],env.time_history[:-1])])
#print(delta_time.mean()*ctrl_fpT,delta_time.mean(),delta_time.var())
env.close()