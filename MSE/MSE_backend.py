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
        self.world = core.World(self.agent_groups,self.cfg)
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
                    data_queue.put([self.world.total_time,self.world.get_state()])
                elif item[0] == 'get_obs':
                    data_queue.put(self.world.get_obs())
                elif item[0] == 'set_state':
                    if item[1][2]:
                        self.world.reset()
                    self.world.set_state(item[1][0],item[1][1])
                elif item[0] == 'set_action':
                    self.world.set_action(item[1][0],item[1][1])
                    

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

    def set_state(self,state,enable_list = None,reset = False):
        if enable_list is None:
            enable_list = [True]* len(state)
        self.common_queue.put(['set_state',(enable_list,state,reset)])

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