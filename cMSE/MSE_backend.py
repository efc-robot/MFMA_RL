import sys 
sys.path.append("../../")
from MF_env import basic

from . import core
from multiprocessing import Process, Queue, Manager
import threading
import time

class MSE_backend(object):
    def __init__(self,scenario,fps = 100.0, dt = 0.1):
        self.agent_groups = scenario['agent_groups']
        self.cfg = {'dt': dt}
        self.use_gui = scenario['common']['use_gui']
        self.fps = fps
        self.manager = Manager()
        self.manager_dict = self.manager.dict()
        self.manager_dict['run'] = 'pause'
        self.common_queue = Queue(maxsize = 5)
        self.data_queue = Queue(maxsize = 5)
        self.sub_process = Process(target = self._start_process,args = (self.manager_dict,self.common_queue,self.data_queue))
        self.sub_process.start()

    def _fps_func(self,t,event,event_stop):
        while True:
            if event_stop.is_set():
                break
            event.set()
            time.sleep(t)
        #t = threading.Thread(target=self._fps_func,args=(t,event))
        #t.start()
        
    def _render_func(self,event,event_stop):
        while True:
            if event_stop.is_set():
                break
            event.set()
            time.sleep(1.0/30.0)
 
    def _start_process(self,manager_dict,common_queue,data_queue):
        self.num = 0 
        for (_,agent_group) in self.agent_groups.items():
            self.num = agent_group[0]['num'] + self.num
        self.world = core.World(self.num,self.cfg['dt'])
        index = 0
        for (_,agent_group) in self.agent_groups.items():
            for agent_prop in agent_group:
                #print(agent_prop)
                R_safe = agent_prop['R_safe']
                R_reach = agent_prop['R_reach']
                L_car = agent_prop['L_car']
                W_car = agent_prop['W_car']
                L_axis = agent_prop['L_axis']
                R_laser = agent_prop['R_laser']
                N_laser = agent_prop['N_laser']
                K_vel = agent_prop['K_vel']
                K_phi = agent_prop['K_phi']
                init_x = agent_prop['init_x']
                init_y = agent_prop['init_y']
                init_theta = agent_prop['init_theta']
                init_vel_b = agent_prop['init_vel_b']
                init_phi = agent_prop['init_phi']
                init_movable = agent_prop['init_movable']
                init_target_x = agent_prop['init_target_x']
                init_target_y = agent_prop['init_target_y']
                self.world.SetWorld(index,R_safe,R_reach,L_car,W_car,L_axis,R_laser,N_laser,K_vel,K_phi,init_x,
                init_y,init_theta,init_vel_b,init_phi,init_movable,init_target_x,init_target_y)
                index = index+1
        #self.world = core.World(self.agent_groups,self.cfg)
        self.fps_stop_event = threading.Event()
        self.render_stop_event = threading.Event()
        if self.fps != 0:
            self.fps_event = threading.Event()
            self.fps_event.set()
            fps_t = threading.Thread(target=self._fps_func,args=(1.0/self.fps,self.fps_event,self.fps_stop_event))
            fps_t.start()
        if self.use_gui:
            self.render_event = threading.Event()
            self.render_event.set()
            render_t = threading.Thread(target=self._render_func,args=(self.render_event,self.render_stop_event))
            render_t.start()
        while True:
            if not common_queue.empty():
                try:
                    item = common_queue.get_nowait()
                except :
                    print('this should not happend')
                if item[0] == 'reset':
                    self.world.reset()
                elif item[0] == 'kill':
                    self.fps_stop_event.set()
                    self.render_stop_event.set()
                    break
                elif item[0] == 'get_state':
                    gstate = []
                    for gstate_idx in range(0,self.num):
                        gstate.append(self.world.get_state(gstate_idx))
                    data_queue.put([self.world.total_time,gstate])
                elif item[0] == 'get_obs':
                    obs = []
                    for obs_idx in range(0,self.num):
                        obs.append(self.world.get_obs(obs_idx))
                    data_queue.put(obs)
                elif item[0] == 'set_state':
                    state_num = len(item[1][1])
                    for state_idx in range(0,state_num):
                        x = item[1][1][state_idx].x
                        y = item[1][1][state_idx].y
                        vel_b = item[1][1][state_idx].vel_b
                        theta = item[1][1][state_idx].theta
                        phi = item[1][1][state_idx].phi
                        movable = item[1][1][state_idx].movable
                        crash = item[1][1][state_idx].crash
                        reach = item[1][1][state_idx].reach
                        target_x = item[1][1][state_idx].target_x
                        target_y = item[1][1][state_idx].target_y
                        self.world.set_state(state_idx,item[1][0][state_idx],x,y,vel_b,theta,phi,movable,crash,reach,target_x,target_y)
                elif item[0] == 'set_action':
                    action_num = len(item[1][1])
                    for action_idx in range(0,action_num):
                        self.world.set_action(action_idx,item[1][0][action_idx],item[1][1][action_idx].ctrl_vel,item[1][1][action_idx].ctrl_phi)
                    

            if manager_dict['run'] == 'pause':
                continue
            elif manager_dict['run'] == 'run':
                if self.fps !=0:
                    if self.fps_event.is_set():
                        self.fps_event.clear()
                        self.world.step()
                else:
                    self.world.step()
            
            if self.use_gui:
                if self.render_event.is_set():
                    self.render_event.clear()
                    self.world.render()

    def get_state(self):
        self.common_queue.put(['get_state',None])
        all_state = self.data_queue.get()
        return all_state

    def set_state(self,state,enable_list = None):
        if enable_list is None:
            enable_list = [True]* len(state)
        self.common_queue.put(['set_state',(enable_list,state)])

    def get_obs(self):
        self.common_queue.put(['get_obs',None])
        obs = self.data_queue.get()
        return obs

    def set_action(self,actions,enable_list= None):
        if enable_list is None:
            enable_list = [True]* len(actions)
        self.common_queue.put(['set_action',(enable_list,actions)])

    def pause(self):
        self.manager_dict['run'] = 'pause'

    def go_on(self):
        self.manager_dict['run'] = 'run'

    def close(self):
        self.common_queue.put(['kill',None])
        self.sub_process.join()