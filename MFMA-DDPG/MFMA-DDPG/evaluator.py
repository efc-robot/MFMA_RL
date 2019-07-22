import numpy as np
import torch
import argparse
import gym.spaces
import gym
import time
import os
import pickle

from MFMA.make_env import make_env
from policy import naive_policy, NN_policy
from multiprocessing import Process, Queue

class Evaluator(object):
    def __init__(self):
        self.env = None
        self.actor = None
        
    def setup(self, env_name, senario, cfg, logger,  num_episodes = 10, multi_process = True):
        self.env_name = env_name
        self.senario = senario
        self.cfg = cfg
        self.logger = logger
        self.num_episodes = num_episodes
        self.multi_process = multi_process
        self.policy_type = None
        self.policy = None
        if self.multi_process :
            self.queue = Queue(maxsize = 1)
            self.sub_process = Process(target = self.start_eval_process,args = (self.queue,))
            self.sub_process.start()
        else :
            self.setup_gym_env()
            
    def setup_gym_env(self):
        self.env = make_env(self.env_name,self.senario,self.cfg)

    def start_eval_process(self,queue):
        self.setup_gym_env()
        while True:
            item = queue.get(block = True)
            if item[0] == 'close':
                break
            elif item[0] == 'update':
                self.update_policy(item[1])
            elif item[0] == 'eval':
                self.run_eval(item[1])

    def update_policy(self,policy_args = None):
        if self.policy_type == 'naive':
            self.policy = naive_policy(policy_args)
        elif self.policy_type == 'NN':
            self.policy = NN_policy(policy_args)
        pass
        
    def run_eval(self,total_cycle):
        assert self.actor is not None
        observation = None
        result = []
        for episode in range(self.num_episodes):

            observation = self.env.reset()
            all_done = False
            timedone = False
            while not (all_done or timedone):
                action = self.policy.inference(observation)
                observation, reward, done, info = self.env.step(action)
                all_done = True
                for done_ in done:
                    all_done = all_done and done_
                timedone = info['time_end']
                
             result.append(episode_reward)

        result = np.array(result).reshape(-1,1)
        result_mean = result.mean()
        result_std = result.std(ddof = 1)
        if self.logger is not None :
            self.logger.trigger_log( 'eval_reward_mean',result_mean, total_cycle)
            self.logger.trigger_log( 'eval_reward_std',result_std, total_cycle)
        localtime = time.asctime( time.localtime(time.time()) )
        print("{} eval : cycle {:<5d}\treward mean {:.2f}\treward std {:.2f}".format(localtime,total_cycle,result_mean,result_std))
        
    def trigger_update_policy(self, policy_args = None):
        if self.multi_process :
            self.queue.put(('update',policy_args),block = True)
        else:
            self.update_policy(policy_args)

    def trigger_eval_process(self,total_cycle):
        if self.multi_process :
            self.queue.put(('eval',total_cycle),block = True)
        else :
            self.run_eval(total_cycle)

    def trigger_close(self):
        if self.multi_process :
            self.queue.put(('close',None),block = True)

    def __del__(self):
        if self.env is not None:
            self.env.close()
    
    
Singleton_evaluator = Evaluator()


if __name__ == "__main__":

    parser = argparse.ArgumentParser(description='Eval DDPG')
    parser.add_argument('--logdir', default=None, type=str, help='result output dir')
    parser.add_argument('--env', default=None, type=str, help='open-ai gym environment')
    parser.add_argument('--model-dir', default=None, type=str, help='actor for evaluation')
    parser.add_argument('--num-episodes', default=10, type=int, help='number of episodes')
    parser.add_argument('--visualize', dest='visualize', action='store_true',help='enable render in evaluation progress')
    parser.set_defaults(visualize=False)
    
    args = parser.parse_args()
    if args.logdir is not None:
        with open(args.logdir,'rb') as f:
            exp_args = pickle.load(f)
            args.env = exp_args.env
            args.model_dir = exp_args.result_dir
            
    assert args.env is not None
    assert args.model_dir is not None
    
    Singleton_evaluator.setup(env_name = args.env,
                              logger = None,
                              num_episodes = 10,
                              model_dir = args.model_dir,
                              multi_process = False,
                              visualize = args.visualize,
                              rand_seed = 0)
                              
    Singleton_evaluator.load_from_file()
    Singleton_evaluator.run_eval(0)
