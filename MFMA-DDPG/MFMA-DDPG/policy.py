import torch
import math
import numpy as np
import os

class NN_policy(object):
    def __init__(self,policy_args):
        self.model_dir = policy_args['model_dir']
        self.actor = torch.load(os.path.join(self.model_dir,'actor.pkl'))
    
    def inference(self,obs):
        with torch.no_grad():
            action = self.actor(obs).cpu().numpy()
            action = np.clip(action, -1., 1.)
        return action


class naive_policy(object):
    def __init__(self,policy_args):
        self.max_phi = policy_args['max_phi']
        self.l = policy_args['l']
        self.dist = policy_args['dist']
        self.min_r = self.l/np.tan(self.max_phi)
        self.right_o = np.array([self.min_r,0.0])
        self.left_o = np.array([-self.min_r,0.0])
    
    def inference(self,obs_list):
        obs_list = obs_list[0]
        #if isinstance(obs_list,torch.Tensor):
        #    obs_list = obs_list.tolist()
        action_list = []
        for obs in obs_list:
            theta = obs[2]
            xt = obs[3] - obs[0]
            yt = obs[4] - obs[1]
            xt,yt = (xt*np.cos(theta)+yt*np.sin(theta),yt*np.cos(theta)-xt*np.sin(theta))

            if abs(yt) < self.dist:
                vel = np.sign(xt)
                phi = 0
            else:
                in_min_r = (xt**2+(abs(yt)-self.min_r)**2)< self.min_r**2
                vel = -1 if (in_min_r ^ xt<0) else 1
                phi = -1 if (in_min_r ^ yt<0) else 1

            action_list.append([vel,phi])
        return action_list