import numpy as np
import os
import pickle
import torch
import gym
from copy import deepcopy
from .arguments import Singleton_arger
from .logger import Singleton_logger
from .ddpg import DDPG
import sys
sys.path.append("..")
from MF_env import policy

class DDPG_trainer(object):
    def __init__(self):
        train_args = Singleton_arger()['train']
        self.nb_epoch = train_args['nb_epoch']
        self.nb_cycles_per_epoch = train_args['nb_cycles_per_epoch']
        self.nb_rollout_steps = train_args['nb_rollout_steps']
        self.nb_train_steps = train_args['nb_train_steps']
        self.nb_warmup_steps = train_args['nb_warmup_steps']
        self.train_mode = train_args['train_mode']
        
    def setup(self,env_instance,agent,memory):
        main_args = Singleton_arger()['main']
        Singleton_logger.setup(main_args['result_dir'])
        self.env = env_instance
        self.agent = agent
        self.memory = memory
        self.result_dir = main_args['result_dir']
    
    def train(self):
        for epoch in range(self.nb_epoch):
            for cycle in range(self.nb_cycles_per_epoch):
                self.env.reset_rollout()
                rollout_policy = policy.NN_policy(self.agent.actor)
                self.env.rollout(rollout_policy.inference, 10)
                trajectoy = self.env.get_trajectoy()
                for traj in (trajectoy):
                    for idx_agent in range(len(traj['done'])):
                        self.memory.append( [traj['obs'][idx_agent].pos,traj['obs'][idx_agent].laser_data], 
                                            [traj['action'][idx_agent].ctrl_vel,traj['action'][idx_agent].ctrl_phi],
                                            traj['reward'][idx_agent],
                                            [traj['obs_next'][idx_agent].pos,traj['obs_next'][idx_agent].laser_data],
                                            traj['done'][idx_agent])
                print(epoch,cycle,self.memory._nb_entries)