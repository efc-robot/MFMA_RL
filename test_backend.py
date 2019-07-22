from MSE import MSE_backend
from scenario.paser import  parse_senario 
from MF_env import basic
import time
scenario = parse_senario('./scenario/scenario.yaml')

back_end = MSE_backend.MSE_backend(scenario)
state_list = []
for agent in scenario['agent_groups']['group_1']:
    state = basic.AgentState()
    state.x = agent['init_x']
    state.y = agent['init_y']
    state.theta = agent['init_theta']
    state.target_x = agent['init_target_x']
    state.target_y = agent['init_target_y']
    state.vel_b = agent['init_vel_b']
    state.phi = agent['init_phi']
    state.movable = agent['init_movable']
    state.crash = False
    state.reach = False
    state_list.append(state)
    
back_end.set_state(state_list)
back_end.go_on()
action = basic.Action()
action.ctrl_vel = 0.1
action.ctrl_phi = 0.22
back_end.set_action([action,action,action,action])
time.sleep(10)
back_end.close()