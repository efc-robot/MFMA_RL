from MF_env import MultiFidelityEnv
from MSE import MSE_backend
from MF_env import basic
from MFMA_DDPG.trainer import DDPG_trainer
from MFMA_DDPG.memory import Memory
from MFMA_DDPG.ddpg import DDPG
from MFMA_DDPG.arguments import Singleton_arger
from scenario.paser import  parse_senario
import torch

import time
import numpy as np
import math



fps = 100.0
dt = 0.02
ctrl_fps = 20.0

eval_scenario = parse_senario('./scenario/scenario_eval.yaml')
eval_back_end = MSE_backend.MSE_backend(eval_scenario,fps,dt)
eval_env = MultiFidelityEnv.MultiFidelityEnv(eval_scenario,eval_back_end)

scenario = parse_senario('./scenario/scenario.yaml')
back_end = MSE_backend.MSE_backend(scenario,fps,dt)
env = MultiFidelityEnv.MultiFidelityEnv(scenario,back_end)
memory = Memory(int(1e5),(2,),[(5,),(32,)])
agent = DDPG(Singleton_arger()['agent'])
agent.setup(5,32,2,Singleton_arger()['model'])
trainer = DDPG_trainer()
trainer.setup(env,eval_env,agent,memory)
trainer.train()
#env.reset_rollout()
#env.rollout(n_policy.inference,ctrl_fps,check_done)
#trajectoy = env.get_trajectoy()
#print(trajectoy[0])
env.close()