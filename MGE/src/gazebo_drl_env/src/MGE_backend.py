#!/usr/bin/env python
import tf
import os
from gazebo_msgs.srv import*
from sensor_msgs.msg import*
from geometry_msgs.msg import*
from nav_msgs.msg import*
from gazebo_drl_env.srv import*
from gazebo_drl_env.msg import*
import numpy as np
import threading
import thread
import time
import math
import core
import roslaunch
import subprocess
import rospy


class MGE_backend(object):
    '''def __init__(self,scenario):
        rospy.init_node('get_state')'''
    def __init__(self,scenario):
        self.scenario = scenario
        self.l_axis = 0.288
        self.AGENT_NUMBER = self.scenario['agent_groups']['group_1'][0]['num']
        self.event = threading.Event()
        self.event.set()#get True
        rospy.init_node('core')
        self.mycore = core.core(scenario)
        self.get_core = core.core(scenario)
        self.thread1 = core.myThread(self.scenario,1,'get_vel', 1, self.event, self.get_core)
        self.thread2 = core.myThread(self.scenario,2,'set_act', 2, self.event, self.mycore)
        self.thread3 = core.myThread(self.scenario,3,'get_laser', 3, self.event, self.get_core)
        self.thread1.start()
        self.thread2.start()
        self.thread3.start()

    def get_state(self):
        state = self.get_core.send_state()
        time = state[0].sim_time
        state_list = state[1]
        return [time,state_list]

    def get_obs(self):
        obs = self.get_core.send_obs()
        return obs

    def set_state(self,states,enable_list = None, reset = False):
        if enable_list is None:
            enable_list = [True]* len(states)
        for i in range(self.AGENT_NUMBER):
            if enable_list[i] == True:
                self.mycore.x[i] = states[i]['(x,y)'][0]
                self.mycore.y[i] = states[i]['(x,y)'][1]
                self.mycore.xt[i] = states[i]['(xt,yt)'][0]
                self.mycore.yt[i] = states[i]['(xt,yt)'][1]
                self.mycore.theta[i] = states[i]['theta']
                self.mycore.vb[i] = states[i]['vb']
                self.mycore.phi[i] = states[i]['phi']
                self.mycore.moveable[i] = states[i]['moveable']
                self.mycore.crash[i] = states[i]['crash']
                self.mycore.reach[i] = states[i]['reach']

        #self.thread4 = core.myThread(self.scenario,4,'set_state',4, self.mycore)
        if reset == False:
            self.mycore.set_state()
            return True
        elif reset == True and False not in enable_list:
            self.mycore.reset_sim()
            self.mycore.set_state()
            return True
        else:
            print('False command!!')
            return False

    def set_action(self,actions,enable_list = None):
        if enable_list is None:
            enable_list = [True]* len(actions)
        for i in range(self.AGENT_NUMBER):
            if enable_list[i] == True:
                self.mycore.vb[i] = actions[i]['vb']
                self.mycore.phi[i] = actions[i]['phi']
        return True

    def pause(self):
        #pause
        self.mycore.pause()

    def go_on(self):
        #go_on
        self.mycore.go_on()

    def close(self):
        self.event.clear()