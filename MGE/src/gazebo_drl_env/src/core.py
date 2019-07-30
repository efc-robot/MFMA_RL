#!/usr/bin/env python
#coding:utf-8

import rospy
from gazebo_msgs.srv import*
from sensor_msgs.msg import*
from geometry_msgs.msg import*
from nav_msgs.msg import*
from gazebo_drl_env.srv import agent_state
from gazebo_drl_env.srv import agent_vel
from gazebo_drl_env.srv import SimpleCtrl
from gazebo_drl_env.srv import agent_laser
from gazebo_drl_env.srv import agent_obs
from gazebo_drl_env.msg import control_group_msgs
from gazebo_drl_env.msg import state_group_msgs
from gazebo_drl_env.msg import control_msgs
from gazebo_drl_env.msg import state_msgs
from gazebo_drl_env.msg import laser_group_msgs
from std_srvs.srv import Empty
import os
import tf
import numpy as np
import threading
import thread
import time
import math

#AGENT_NUMBER=4
#l_axi=1

class core:
    def __init__(self,scenario):
        self.scenario = scenario
        self.l_axis = 0.288
        self.AGENT_NUMBER = self.scenario['agent_groups']['group_1'][0]['num']
        self.R_safe = self.scenario['agent_groups']['group_1'][0]['R_safe']
        self.R_reach = self.scenario['agent_groups']['group_1'][0]['R_reach']
        self.x = [0.0]*self.AGENT_NUMBER
        self.y = [0.0]*self.AGENT_NUMBER
        self.theta = [0.0]*self.AGENT_NUMBER
        self.vb = [0.0]*self.AGENT_NUMBER
        self.phi = [0.0]*self.AGENT_NUMBER
        self.vb_tem = [0.0]*self.AGENT_NUMBER
        self.phi_tem = [0.0]*self.AGENT_NUMBER
        self.xt = [0.0]*self.AGENT_NUMBER
        self.yt = [0.0]*self.AGENT_NUMBER
        self.moveable = [True]*self.AGENT_NUMBER
        self.crash = [False]*self.AGENT_NUMBER
        self.reach = [False]*self.AGENT_NUMBER
        self.laser = [LaserScan()]*self.AGENT_NUMBER
        self.laser_tem = [LaserScan()]*self.AGENT_NUMBER

    def set_action(self,event):
        linear = [0.0]*self.AGENT_NUMBER
        angular = [0.0]*self.AGENT_NUMBER
        pub_vel = [0.0]*self.AGENT_NUMBER
        for i in range(self.AGENT_NUMBER):
            pub_vel[i] = rospy.Publisher('/agent_'+str(i+1)+'/cmd_vel',Twist,queue_size = 1)
        rate = rospy.Rate(1)
        while event.isSet():
            for i in range(self.AGENT_NUMBER):
                linear[i] = Vector3(self.vb[i],0.0,0.0)
                #print('set_vb')
                #print(self.vb[i])
                if self.phi[i] == 0.0:
                    angular[i] = Vector3(0.0,0.0,0.0)
                else:
                    angular[i] = Vector3(0.0,0.0,2.0*self.vb[i]/(self.l_axis*(-1.0+2.0/math.tan(self.phi[i]))))
                pub_vel[i].publish(Twist(linear[i],angular[i]))
            rate.sleep()

    def set_state(self):
        #set state to gazebo
        rospy.wait_for_service('gazebo/set_model_state')
        try:
            state_1 = rospy.ServiceProxy('gazebo/set_model_state', SetModelState)
            model = SetModelStateRequest()
            # set x,y,xt,yt,theta=yaw
            for i in range(self.AGENT_NUMBER):
                model.model_state.model_name = 'agent_'+str(i+1)
                model.model_state.pose.position.x = self.x[i]
                #print('****')
                #print(model.model_state.pose.position.x)
                model.model_state.pose.position.y = self.y[i]
                (orientation_x,orientation_y,orientation_z,orientation_w) = tf.transformations.quaternion_from_euler(0.0,0.0,self.theta[i])
                model.model_state.pose.orientation.x = orientation_x
                model.model_state.pose.orientation.y = orientation_y
                model.model_state.pose.orientation.z = orientation_z
                model.model_state.pose.orientation.w = orientation_w
                #(tr,tp,ty) = tf.transformations.euler_from_quaternion([model.model_state.pose.orientation.x,model.model_state.pose.orientation.y,model.model_state.pose.orientation.z,model.model_state.pose.orientation.w])
                #print('yaw=:%f' %ty)
                state_1(model)
            for i in range(self.AGENT_NUMBER):
                model.model_state.model_name = 'agent_'+str(i+1)+'_target'
                model.model_state.pose.position.x = self.xt[i]
                model.model_state.pose.position.y = self.yt[i]
                resp_ = state_1(model)
        except rospy.ServiceException,e:
            rospy.logwarn('Service call failed:%s' %e)

    def send_obs(self):
        rospy.wait_for_service('gazebo/get_model_state')
        try:
            state_1 = rospy.ServiceProxy('gazebo/get_model_state', GetModelState)
            model = GetModelStateRequest()
            # get x,y,xt,yt,theta=yaw
            for i in range(self.AGENT_NUMBER):
                model.model_name = 'agent_'+str(i+1)
                obs = state_1(model)
                self.x[i] = obs.pose.position.x
                self.y[i] = obs.pose.position.y
                (r,p,y) = tf.transformations.euler_from_quaternion([obs.pose.orientation.x,obs.pose.orientation.y,obs.pose.orientation.z,obs.pose.orientation.w])
                self.theta[i] = y
                #print('yaw=:%f' %self.theta[i])

            for i in range(self.AGENT_NUMBER):
                model.model_name = 'agent_'+str(i+1)+'_target'
                obs_target = state_1(model)
                self.xt[i] = obs_target.pose.position.x
                self.yt[i] = obs_target.pose.position.y

            '''rospy.wait_for_service('agent_laser')
            state_laser = rospy.ServiceProxy('agent_laser',agent_laser)
            obs_laser = state_laser(True)
            #print(type(obs_laser))
            self.laser = obs_laser.all_laser'''

            obs_list = []
            for i in range(self.AGENT_NUMBER):
                new_dict = {
                'x':self.x[i],'y':self.y[i],
                'theta':self.theta[i],
                'xt':self.xt[i],'yt':self.yt[i],
                'laser':self.laser[i]}
                obs_list.append(new_dict)
            return obs_list

        except rospy.ServiceException,e:
            rospy.logwarn('Service call failed:%s' %e)

    def send_state(self):
        #rospy.init_node('get_state')
        rospy.wait_for_service('gazebo/get_model_state')
        try:
            state_1 = rospy.ServiceProxy('gazebo/get_model_state', GetModelState)
            model = GetModelStateRequest()
            # get x,y,xt,yt,theta=yaw
            for i in range(self.AGENT_NUMBER):
                model.model_name = 'agent_'+str(i+1)
                obs = state_1(model)
                self.x[i] = obs.pose.position.x
                self.y[i] = obs.pose.position.y
                (r,p,y) = tf.transformations.euler_from_quaternion([obs.pose.orientation.x,obs.pose.orientation.y,obs.pose.orientation.z,obs.pose.orientation.w])
                self.theta[i] = y
                #print('yaw=:%f' %self.theta[i])

            '''for i in range(self.AGENT_NUMBER):
                self.pos.append([self.x[i],self.y[i]])'''
            for i in range(self.AGENT_NUMBER):
                model.model_name = 'agent_'+str(i+1)+'_target'
                obs_target = state_1(model)
                self.xt[i] = obs_target.pose.position.x
                self.yt[i] = obs_target.pose.position.y
            '''for i in range(self.AGENT_NUMBER):
                self.pos_t.append([self.xt[i],self.yt[i]])'''
            # get crash
            for i in range(self.AGENT_NUMBER):
                for j in range(i+1,self.AGENT_NUMBER):
                    if self.near(self.x[i],self.y[i],self.x[j],self.y[j],self.R_safe) == True:
                        self.crash[i]=True
                        self.crash[j]=True
            # get reach
            for i in range(self.AGENT_NUMBER):
                if self.near(self.x[i],self.y[i],self.xt[i],self.yt[i],self.R_reach) == True:
                    self.reach[i]=True
            # get moveable 
            for i in range(self.AGENT_NUMBER):
                if self.reach[i] or self.crash[i] == True:
                    self.moveable[i]=True

            #get vb,phi
            '''rospy.wait_for_service('agent_vel')
            state_vel = rospy.ServiceProxy('agent_vel',agent_vel)
            obs_vel = state_vel(True)
            self.vb = list(obs_vel.vb)
            self.phi = list(obs_vel.phi)'''

            #get sim_time
            rospy.wait_for_service('/gazebo/get_world_properties')
            sim_time = rospy.ServiceProxy('/gazebo/get_world_properties',GetWorldProperties)
            obs_time = sim_time()

            state_list = []
            for i in range(self.AGENT_NUMBER):
                new_dict = {
                'x':self.x[i],'y':self.y[i],'theta':self.theta[i],
                'vb':self.vb[i],'phi':self.phi[i],
                'xt':self.xt[i],'yt':self.yt[i],
                'moveable':self.moveable[i],'crash':self.crash[i],
                'reach':self.reach[i]}
                state_list.append(new_dict)

            return [obs_time,state_list]

        except rospy.ServiceException,e:
            rospy.logwarn('Service call failed:%s' %e)

    def get_laser(self,event):
        for i in range(self.AGENT_NUMBER):
            rospy.Subscriber('agent_'+str(i+1)+'/scan',LaserScan,self.callback_laser)
        '''time.sleep(0.1)
        return_srv = rospy.Service('agent_laser',agent_laser,self.handle_func_laser)
        rospy.loginfo('Ready to handle the request')'''
        while event.isSet():
            '''for i in range(self.AGENT_NUMBER):
                rospy.Subscriber('agent_'+str(i+1)+'/odom',Odometry,self.callback)'''
            #time.sleep(0.1)
            #rospy.loginfo('Ready to handle the request')
        #rospy.spin()

    def get_vel(self,event):
        for i in range(self.AGENT_NUMBER):
            rospy.Subscriber('agent_'+str(i+1)+'/odom',Odometry,self.callback)
        '''time.sleep(0.1)
        return_srv = rospy.Service('agent_vel',agent_vel,self.handle_func_vel)
        rospy.loginfo('Ready to handle the request')'''
        while event.isSet():
            '''for i in range(self.AGENT_NUMBER):
                rospy.Subscriber('agent_'+str(i+1)+'/odom',Odometry,self.callback)'''
            #time.sleep(0.1)
            #rospy.loginfo('Ready to handle the request')
        #rospy.spin()

    def callback(self,Odometry):
        agentID = int(Odometry.header.frame_id[6])
        self.vb[agentID-1]=Odometry.twist.twist.linear.x
        if Odometry.twist.twist.linear.x<=0.01 or Odometry.twist.twist.angular.z<=0.01:
            self.phi[agentID-1]=0.0
        else:
            self.phi[agentID-1]=math.atan(2.0/(1.0+2.0*Odometry.twist.twist.linear.x/(Odometry.twist.twist.angular.z*self.l_axis)))

    def callback_laser(self,LaserScan):
        agentID = int(LaserScan.header.frame_id[6])
        self.laser[agentID-1]=LaserScan
        #print(type(self.laser_tem[0]))

    def pause(self):
        rospy.wait_for_service('/gazebo/pause_physics')
        pause_physics_client = rospy.ServiceProxy('/gazebo/pause_physics',Empty)
        pause_physics_client()

    def go_on(self):
        rospy.wait_for_service('/gazebo/unpause_physics')
        unpause_physics_client = rospy.ServiceProxy('/gazebo/unpause_physics',Empty)
        unpause_physics_client()

    def reset_sim(self):
        #reset simulation
        rospy.wait_for_service('/gazebo/reset_simulation')
        reset_sim = rospy.ServiceProxy('/gazebo/reset_simulation',Empty)
        obs2 = reset_sim()

    def reset_world(self):
        #reset simulation
        rospy.wait_for_service('/gazebo/reset_world')
        reset_sim = rospy.ServiceProxy('/gazebo/reset_world',Empty)
        obs2 = reset_sim()

    def handle_func_laser(self,req):
        if req.request == True:
            rospy.loginfo('Success sending')
        return [self.laser_tem]

    def handle_func_vel(self,req):
        if req.request == True:
            rospy.loginfo('Success sending')
        return [self.vb_tem,self.phi_tem]

    def near(self,pos1_x,pos1_y,pos2_x,pos2_y,min_distance):
        vector1 = np.array([pos1_x,pos1_y])
        vector2 = np.array([pos2_x,pos2_y])
        if np.linalg.norm(vector1-vector2) <= min_distance:
            return True
        else:
            return False

class myThread(threading.Thread):
    def __init__(self, scenario, threadID, name, actionID, event, mycore = None):
        threading.Thread.__init__(self)
        self.threadID = threadID
        self.name = name
        self.actionID = actionID
        self.result = []
        self.result_obs = []
        self.scenario = scenario
        self.mycore = mycore
        self.event = event
    def run(self):
        #core_node = core(self.scenario)
        if self.actionID == 1:
            self.mycore.get_vel(self.event)
        elif self.actionID == 2:
            self.mycore.set_action(self.event)
        elif self.actionID == 3:
            self.mycore.get_laser(self.event)
   