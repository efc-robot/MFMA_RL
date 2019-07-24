import torch
import math
import numpy as np
import os
import io
import copy
import ctypes
from . import basic

class NN_policy(object):
    def __init__(self,policy_args):
        self.actor = copy.deepcopy(policy_args)
    
    def inference(self,obs_list):
        with torch.no_grad():
            pos = torch.Tensor(np.vstack([obs.pos for obs in obs_list])).cuda()
            laser_data = torch.Tensor(np.vstack([obs.laser_data for obs in obs_list])).cuda()
            
            action = self.actor(pos,laser_data).cpu().numpy()
            action = np.clip(action, -1., 1.)
        action_list = []
        for idx in range(action.shape[0]):
            a = basic.Action()
            a.ctrl_vel = action[idx,0]
            a.ctrl_phi = action[idx,1]
            action_list.append(a)
        return action_list
        


class naive_policy(object):
    def __init__(self,max_phi,l,dist):
        self.max_phi = max_phi
        self.l = l
        self.dist = dist
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


class RVO_policy(object):
    def __init__(self):
        self.max_phi = math.pi/6.0
        self.l = 0.4
        self.dist = 0.1
        self.min_r = self.l/np.tan(self.max_phi)
        self.right_o = np.array([self.min_r,0.0])
        self.left_o = np.array([-self.min_r,0.0])
        self.function = ctypes.CDLL('./libRVO.so')._Z11AgentNewVeliPKfS0_PfS1_S0_S0_S0_S0_fff
    def inference(self,obs_list):
        agent_num = len(obs_list)
        x_position = []
        y_position = []
        velocity_x = []
        velocity_y = []
        prefer_vel_x = []
        prefer_vel_y = []

        for idx in range(agent_num):
            obs = obs_list[idx].pos
            x_position.append(obs[0])
            y_position.append(obs[1])
            velocity_x.append(math.cos(obs[2]))
            velocity_y.append(math.sin(obs[2]))
            xt = obs[3] - obs[0]
            yt = obs[4] - obs[1]
            tnorm = (xt**2+yt**2)**0.5
            if tnorm<1:
                xt/=tnorm
                yt/=tnorm
            prefer_vel_x.append(xt)
            prefer_vel_y.append(yt)
        cpp_x_position = (ctypes.c_float * agent_num)(*x_position)
        cpp_y_position = (ctypes.c_float * agent_num)(*y_position)
        cpp_velocity_x = (ctypes.c_float * agent_num)(*velocity_x)
        cpp_velocity_y = (ctypes.c_float * agent_num)(*velocity_y)
        cpp_prefer_vel_x = (ctypes.c_float * agent_num)(*prefer_vel_x)
        cpp_prefer_vel_y = (ctypes.c_float * agent_num)(*prefer_vel_y)
        time_Horizon = ctypes.c_float(0.3)
        neighbor_Dist = ctypes.c_float(10.0)
        timestep = ctypes.c_float(0.1)
        maxspeed = [1.0,]*agent_num
        radius = [0.32,]*agent_num
        cpp_radius = (ctypes.c_float * agent_num)(*radius)
        cpp_maxspeed = (ctypes.c_float * agent_num)(*maxspeed) 
        self.function( agent_num, cpp_x_position, cpp_y_position, cpp_velocity_x, cpp_velocity_y, cpp_prefer_vel_x, cpp_prefer_vel_y, cpp_radius, cpp_maxspeed, time_Horizon, neighbor_Dist, timestep)
        temp_velocity_x = np.array(cpp_velocity_x)*1.0
        temp_velocity_y = np.array(cpp_velocity_y)*1.0
        velocity_x = temp_velocity_x.tolist()
        velocity_y = temp_velocity_y.tolist()


        action_list = []
        for idx,obs in enumerate(obs_list):
            vel_norm = (velocity_x[idx]**2 + velocity_y[idx]**2)**0.5
            obs = obs.pos
            theta = obs[2]
            xt = velocity_x[idx]*1.5
            yt = velocity_y[idx]*1.5
            xt,yt = (xt*np.cos(theta)+yt*np.sin(theta),yt*np.cos(theta)-xt*np.sin(theta))
    
            if abs(yt) < self.dist:
                vel = np.sign(xt)
                phi = 0
            else:
                in_min_r = (xt**2+(abs(yt)-self.min_r)**2)< self.min_r**2
                vel = -1 if np.bitwise_xor(in_min_r,xt<0) else 1
                phi = -1 if np.bitwise_xor(in_min_r,yt<0) else 1
            a = basic.Action()
            a.ctrl_vel = vel*vel_norm
            a.ctrl_phi = phi
            action_list.append(a)
        return action_list