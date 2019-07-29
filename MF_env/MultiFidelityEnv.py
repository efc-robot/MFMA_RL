from  gym import Env
import copy
import numpy as np
import math
import threading
import time
from itertools import product
from .basic import Agent,Action,AgentState

class MultiFidelityEnv(Env):
    def __init__(self,senario_dict,backend):
        self.backend = backend
        self.senario_dict = senario_dict
        self.time_limit = senario_dict['common']['time_limit']
        self.reward_coef = senario_dict['common']['reward_coef']
        self.reset_mode = senario_dict['common']['reset_mode']
        self.field_range = senario_dict['common']['field_range']
        self.ref_state_list = []
        self.ref_agent_list = []
        self.agent_num = 0
        for (_,grop) in self.senario_dict['agent_groups'].items():
            for agent_prop in grop:
                agent = Agent(agent_prop)
                state = AgentState()
                state.x = agent.init_x
                state.y = agent.init_y
                state.theta = agent.init_theta
                state.vel_b = agent.init_vel_b
                state.movable = agent.init_movable
                state.phi = agent.init_phi
                state.target_x = agent.init_target_x
                state.target_y = agent.init_target_y
                state.crash = False
                state.reach = False
                self.ref_state_list.append(state)
                self.ref_agent_list.append(agent)
                self.agent_num+=1

    def _random_reset(self,new_state):
        state_list  = copy.deepcopy(new_state)
        enable_list = [state.crash|state.reach for  state in state_list]
        enable_tmp = True in enable_list
        
        crash_idx_list = []
        for idx ,state in enumerate(state_list):
            if state.crash: crash_idx_list.append(idx)
                
        if len(crash_idx_list)>0:
            #try random place agent for 20 times
            for idx in crash_idx_list:
                state_list[idx].crash = False
                state_list[idx].movable = True
            for try_time in range(20):
                for idx in crash_idx_list:
                    state_list[idx].x = np.random.uniform(self.field_range[0],self.field_range[1])
                    state_list[idx].y = np.random.uniform(self.field_range[2],self.field_range[3])
                no_conflict = True
                for idx_a,idx_b in product(range(self.agent_num),range(self.agent_num)):
                    if idx_a == idx_b: continue
                    state_a = state_list[idx_a]
                    state_b = state_list[idx_b]
                    agent_dist = ((state_a.x-state_b.x)**2+(state_a.y-state_b.y)**2)
                    agent_size = (self.ref_agent_list[idx_a].R_safe+self.ref_agent_list[idx_b].R_safe)**2
                    no_conflict = agent_dist < agent_size
                    if not no_conflict : break
                if no_conflict: break

        reach_idx_list = []
        for idx ,state in enumerate(state_list):
            if state.reach: reach_idx_list.append(idx)
        if len(reach_idx_list)>0:
            for idx in reach_idx_list:
                state_list[idx].reach = False
                state_list[idx].movable = True
                state_list[idx].target_x = np.random.uniform(self.field_range[0],self.field_range[1])
                state_list[idx].target_y = np.random.uniform(self.field_range[2],self.field_range[3])


        return enable_tmp,state_list,enable_list

    def _random_state(self):
        state = AgentState()
        state.x = np.random.uniform(self.field_range[0],self.field_range[1])
        state.y = np.random.uniform(self.field_range[2],self.field_range[3])
        state.target_x = np.random.uniform(self.field_range[0],self.field_range[1])
        state.target_y = np.random.uniform(self.field_range[2],self.field_range[3])
        state.theta = np.random.uniform(0,math.pi*2)
        state.vel_b = 0
        state.phi = 0
        return state

    def _reset_state(self):
        state_list=copy.deepcopy(self.ref_state_list)
        if self.reset_mode == 'random':
            for idx in range(len(state_list)):
                state_list[idx] = self._random_state()
        return state_list

    def _calc_reward(self,new_state,old_state,delta_time):
        crash = self.reward_coef['crash'] if new_state.crash else 0
        reach = self.reward_coef['reach'] if new_state.reach else 0
        new_dist = ((new_state.x-new_state.target_x)**2+(new_state.y-new_state.target_y)**2)**0.5
        old_dist = ((old_state.x-old_state.target_x)**2+(old_state.y-old_state.target_y)**2)**0.5

        potential = self.reward_coef['potential'] * (old_dist-new_dist)
        time_penalty = self.reward_coef['time_penalty']*delta_time
        reward = crash + reach + potential + time_penalty
        #print(re , crash , reach , potential, time_penalty)
        return reward
    
    def get_state(self):
        return self.backend.get_state()

    def set_state(self,state,enable_list = None):
        if enable_list is None:
            enable_list = [True] * len(state)
        self.backend.set_state(state,enable_list)
    
    def bankend_pause(self):
        self.backend.pause()

    def bankend_go_on(self):
        self.backend.go_on()

    def _fps_func(self,t,event,event_stop):
        while True:
            if event_stop.is_set():
                break
            event.set()
            time.sleep(t)


    def reset_rollout(self):
        self.backend.pause()
        self.backend.set_state(self._reset_state(),[True,]*len(self._reset_state()),True)
        self.state_history = []
        self.obs_history = []
        self.time_history = []
        self.action_history = []

    def rollout(self, policy_call_back, control_fps, finish_call_back = None, pause_call_back = None):
        # setting control_fps timmer threading
        self.fps_event = threading.Event()
        self.fps_stop_event = threading.Event()
        self.fps_event.set()
        fps_t = threading.Thread(target=self._fps_func,args=(1.0/control_fps,self.fps_event,self.fps_stop_event))
        fps_t.start()
        # start backend
        self.backend.go_on()
        while True:
            if self.fps_event.is_set():
                self.fps_event.clear()
                # get new state, new obs, and calculate action 
                total_time,new_state = self.backend.get_state()
                new_obs = self.backend.get_obs()
                action = policy_call_back(new_obs)
                self.state_history.append(new_state)
                self.obs_history.append(new_obs)
                self.time_history.append(total_time)
                # check whether we should pause rollout
                if pause_call_back is not None:
                    if pause_call_back(new_state):
                        self.backend.pause()
                        self.fps_stop_event.set()
                        return 'pause'

                # check whether we should stop one rollout
                finish = False
                if finish_call_back is not None:
                    finish = finish_call_back(new_state)
                finish = finish or (total_time > self.time_limit)
                if finish:
                    self.backend.pause()
                    self.fps_stop_event.set()
                    return 'finish'
                if self.reset_mode == 'random':
                    enable_tmp,state_list,enable_list = self._random_reset(new_state)
                    if enable_tmp:
                        self.backend.set_state(state_list,enable_list)
                self.backend.set_action(action)
                self.action_history.append(action)

    def get_trajectoy(self):
        trajectoy = []
        for idx in range(len(self.action_history)):
            obs = []
            obs_next = []
            done = []
            action = []
            reward = []
            time = self.time_history[idx]
            for agent_idx in range(len(self.state_history[idx+1])):
                if self.state_history[idx][agent_idx].movable :
                    obs.append(self.obs_history[idx][agent_idx])
                    obs_next.append(self.obs_history[idx+1][agent_idx])
                    done.append(not  self.state_history[idx+1][agent_idx].movable)
                    action.append(self.action_history[idx][agent_idx])
                    reward.append(self._calc_reward(self.state_history[idx+1][agent_idx],self.state_history[idx][agent_idx],self.time_history[idx+1]-self.time_history[idx]))
            transition = {'obs':obs,'action':action,'reward': reward, 'obs_next':obs_next, 'done':done, 'time':time}
            trajectoy.append(copy.deepcopy(transition))
        return trajectoy

    def get_result(self):
        vel_list = []
        result = {}
        crash_time = 0
        reach_time = 0
        for list_idx in range(len(self.state_history)):
            state_list = self.state_history[list_idx]
            for state_idx in range(len(state_list)):
                state = state_list[state_idx]
                vel_list.append(abs(state.vel_b))
                if list_idx>0:
                    crash_time += 1 if state.crash and not self.state_history[list_idx-1][state_idx].crash else 0
                    reach_time += 1 if state.reach and not self.state_history[list_idx-1][state_idx].reach else 0
        result['crash_time'] = crash_time
        result['reach_time'] = reach_time
        result['mean_vel'] = sum(vel_list)/len(vel_list)
        result['total_time'] = self.time_history[-1]-self.time_history[0]
        return result

                

    def close(self):
        self.backend.close()