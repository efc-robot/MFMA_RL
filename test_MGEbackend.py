from scenario.paser import  parse_senario 
from MGE.src.gazebo_drl_env.src import MGE_backend
import time
import rospy
import subprocess
import os
import signal


if __name__ == '__main__':
	#gazebo_process = subprocess.Popen('roslaunch gazebo_drl_env 4_agents.launch',shell = True)
	#time.sleep(10)
	state = [
	{'(x,y)':[3.0,3.0],'vb':0.0,'phi':0.0,'theta':0.0,'(xt,yt)':[3.0,3.0],'moveable':True,'crash':False,'reach':False},
	{'(x,y)':[3.0,2.7],'vb':0.0,'phi':0.0,'theta':0.0,'(xt,yt)':[3.0,-3.0],'moveable':True,'crash':False,'reach':False},
	{'(x,y)':[-3.0,3.0],'vb':0.0,'phi':0.0,'theta':0.0,'(xt,yt)':[-3.0,3.0],'moveable':True,'crash':False,'reach':False},
	{'(x,y)':[-3.0,-3.0],'vb':0.0,'phi':0.0,'theta':0.0,'(xt,yt)':[-3.0,-3.0],'moveable':True,'crash':False,'reach':False},
	]
	state1 = [
	{'(x,y)':[2.0,2.0],'vb':0.0,'phi':0.0,'theta':0.0,'(xt,yt)':[3.0,3.0],'moveable':True,'crash':False,'reach':False},
	{'(x,y)':[2.0,-2.0],'vb':0.0,'phi':0.0,'theta':0.0,'(xt,yt)':[3.0,-3.0],'moveable':True,'crash':False,'reach':False},
	{'(x,y)':[-2.0,2.0],'vb':0.0,'phi':0.0,'theta':0.0,'(xt,yt)':[-3.0,3.0],'moveable':True,'crash':False,'reach':False},
	{'(x,y)':[-2.0,-2.0],'vb':0.0,'phi':0.0,'theta':0.0,'(xt,yt)':[-3.0,-3.0],'moveable':True,'crash':False,'reach':False},
	]
	action1 = [{'vb':0.25,'phi':0.588},{'vb':0.25,'phi':0.588},{'vb':0.25,'phi':0.588},{'vb':0.25,'phi':0.588}]
	action2 = [{'vb':0.0,'phi':0.0},{'vb':0.0,'phi':0.0},{'vb':0.0,'phi':0.0},{'vb':0.0,'phi':0.0}]
	enable_list = [True,True,True,True]
	scenario = parse_senario('./scenario/scenario.yaml')
	a=MGE_backend.MGE_backend(scenario)
	a.set_state(state1,enable_list,True)
	time.sleep(1)
	num = 0
	while num <= 5:
		print('*****')
		if num%2 == 0:
			a.set_action(action2,enable_list)
			print('stop')
		else:
			a.set_action(action1,enable_list)
			print('run')
		num = num + 1
		#a.pause()
		#a.get_obs()
		time.sleep(0.5)
		print(a.get_state()[1][0]['vb'])
		time.sleep(3)
		#print(a.get_obs()[0])
		#a.go_on()
		#time.sleep(3)
	a.close()
	#while True:
	#	pass
	#print(gazebo_process.pid)
	#os.killpg(gazebo_process.pid, signal.SIGKILL)

