import sys 
sys.path.append("../../")
from MF_env import basic

from . import core
from multiprocessing import Process, Queue, Manager
import threading
import time
import math


#颜色转换，用于gui
def hsv2rgb(h, s, v):
    h = float(h)
    s = float(s)
    v = float(v)
    h60 = h / 60.0
    h60f = math.floor(h60)
    hi = int(h60f) % 6
    f = h60 - h60f
    p = v * (1 - s)
    q = v * (1 - f * s)
    t = v * (1 - (1 - f) * s)
    r, g, b = 0, 0, 0
    if hi == 0: r, g, b = v, t, p
    elif hi == 1: r, g, b = q, v, p
    elif hi == 2: r, g, b = p, v, t
    elif hi == 3: r, g, b = p, q, v
    elif hi == 4: r, g, b = t, p, v
    elif hi == 5: r, g, b = v, p, q
    #r, g, b = int(r * 255), int(g * 255), int(b * 255)
    return (r, g, b)

class MSE_backend(object):
    def __init__(self,scenario,fps = 0.0, dt = 0.1):
        self.agent_groups = scenario['agent_groups']
        self.cfg = {'dt': dt}
        self.use_gui = scenario['common']['use_gui']
        self.fps = fps
        self.cam_range = 4
        self.viewer = None
        self._reset_render()
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
    
    def _reset_render(self):
        self.agent_geom_list = None
    
    def _render(self, mode='human'):
        if self.viewer is None:
            from . import rendering 
            self.viewer = rendering.Viewer(800,800)
        self.agents = []
        for idx in range(self.agent_number):
            agent = self.world.get_agent(idx)
            agent.color = hsv2rgb(360.0/self.agent_number*idx,1.0,1.0)
            self.agents.append(agent)
        # create rendering geometry
        if self.agent_geom_list is None:
            # import rendering only if we need it (and don't import for headless machines)
            from . import rendering
            self.viewer.set_bounds(0-self.cam_range, 0+self.cam_range, 0-self.cam_range, 0+self.cam_range)
            self.agent_geom_list = []
            
                
            for agent in self.agents:
                agent_geom = {}
                total_xform = rendering.Transform()
                agent_geom['total_xform'] = total_xform
                agent_geom['laser_line'] = []

                geom = rendering.make_circle(agent.R_reach)
                geom.set_color(*agent.color)
                xform = rendering.Transform()
                geom.add_attr(xform)
                agent_geom['target_circle']=(geom,xform)

                N = agent.N_laser
                for idx_laser in range(N):
                    theta_i = idx_laser*math.pi*2/N
                    #d = agent.R_laser
                    d = 1
                    end = (math.cos(theta_i)*d, math.sin(theta_i)*d)
                    geom = rendering.make_line((0, 0),end)
                    geom.set_color(0.0,1.0,0.0,alpha = 0.5)
                    xform = rendering.Transform()
                    geom.add_attr(xform)
                    geom.add_attr(total_xform)
                    agent_geom['laser_line'].append((geom,xform))
                
                half_l = agent.L_car/2.0
                half_w = agent.W_car/2.0
                geom = rendering.make_polygon([[half_l,half_w],[-half_l,half_w],[-half_l,-half_w],[half_l,-half_w]])
                geom.set_color(*agent.color,alpha = 0.4)
                xform = rendering.Transform()
                geom.add_attr(xform)
                geom.add_attr(total_xform)
                agent_geom['car']=(geom,xform)

                geom = rendering.make_line((0,0),(half_l,0))
                geom.set_color(1.0,0.0,0.0,alpha = 1)
                xform = rendering.Transform()
                geom.add_attr(xform)
                geom.add_attr(total_xform)
                agent_geom['front_line']=(geom,xform)
                
                geom = rendering.make_line((0,0),(-half_l,0))
                geom.set_color(0.0,0.0,0.0,alpha = 1)
                xform = rendering.Transform()
                geom.add_attr(xform)
                geom.add_attr(total_xform)
                agent_geom['back_line']=(geom,xform)

                self.agent_geom_list.append(agent_geom)

            self.viewer.geoms = []
            for agent_geom in self.agent_geom_list:
                self.viewer.add_geom(agent_geom['target_circle'][0])
                for geom in agent_geom['laser_line']:
                    self.viewer.add_geom(geom[0])
                self.viewer.add_geom(agent_geom['car'][0])
                self.viewer.add_geom(agent_geom['front_line'][0])
                self.viewer.add_geom(agent_geom['back_line'][0])
        self.world.update_laser_state()
        for agent,agent_geom in zip(self.agents,self.agent_geom_list):
            
            for idx,laser_line in enumerate(agent_geom['laser_line']):
                    laser_line[1].set_scale(agent.laser_state[idx],agent.laser_state[idx]) 
            agent_geom['front_line'][1].set_rotation(agent.state.phi)
            agent_geom['target_circle'][1].set_translation(agent.state.target_x,agent.state.target_y)
            agent_geom['total_xform'].set_rotation(agent.state.theta)
            agent_geom['total_xform'].set_translation(agent.state.x,agent.state.y)
            
        return self.viewer.render(return_rgb_array = mode=='rgb_array')


    def _start_process(self,manager_dict,common_queue,data_queue):
        self.num = 0 
        for (_,agent_group) in self.agent_groups.items():
            self.num = agent_group[0]['num'] + self.num   #计算agent数量
        self.world = core.World(self.num,self.cfg['dt'])
        index = 0
        #对每个agent调用core中的SetWorld()分别初始化
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
        self.agent_number = index
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
                    #将c中的AgentState类拷贝到python中的AgentState类，因为部分python函数不能处理c类型
                    for gstate_idx in range(0,self.num):
                        state_c = self.world.get_state(gstate_idx)
                        state_py = basic.AgentState()
                        state_py.x = state_c.x
                        state_py.y = state_c.y
                        state_py.vel_b = state_c.vel_b
                        state_py.theta = state_c.theta
                        state_py.phi = state_c.phi
                        state_py.movable = state_c.movable
                        state_py.crash = state_c.crash
                        state_py.reach = state_c.reach
                        state_py.target_x = state_c.target_x
                        state_py.target_y = state_c.target_y
                        gstate.append(state_py)
                    data_queue.put([self.world.total_time,gstate])
                elif item[0] == 'get_obs':
                    #将c中Observation的类拷贝到python中的Observation类，因为部分python函数不能处理c类型
                    obs = []
                    for obs_idx in range(0,self.num):
                        obs_c = self.world.get_obs(obs_idx)
                        obs_py = basic.Observation()
                        obs_py.pos = [obs_c.pos_x,obs_c.pos_y,obs_c.pos_theta,obs_c.pos_target_x,obs_c.pos_target_y]
                        for i in range(0,N_laser):
                            obs_py.laser_data.append(obs_c.laser_data[i])
                        obs.append(obs_py)
                    data_queue.put(obs)
                elif item[0] == 'set_state':
                    state_num = len(item[1][1])
                    #将前端传过来的字典解包，把agent的所有属性都设为set_state()的参数
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
                    self._render()

    def get_state(self):
        self.common_queue.put(['get_state',None])
        all_state = self.data_queue.get()
        return all_state
    
    def set_state(self,state,enable_list = None,reset = False):
        if reset :
            self._reset_render()
        if enable_list is None:
            enable_list = [True]* len(state)
        self.common_queue.put(['set_state',(enable_list,state,reset)])

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