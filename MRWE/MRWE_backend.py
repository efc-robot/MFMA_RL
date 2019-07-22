import sys 
sys.path.append("../../")
from MF_env import basic


class MRWE_backend(object):
    def __init__(self,scenario):
        # set up the core of your environment
        pass

    def get_state(self):
        # return the state list of your simulator
        time = 0 #simulator time
        n = 1 #agent number
        state_list = [basic.AgentState(),]*n
        return [time,state_list]

    def set_state(self,states,enable_list = None):
        if enable_list is None:
            enable_list = [True]* len(states)
        #set i_th agent state as states[i] if enable_list[i] is True
        return True

    def get_obs(self):
        n = 1 #agent number
        obs_list = [basic.Observation]*n
        return obs_list

    def set_action(self,actions,enable_list= None):
        if enable_list is None:
            enable_list = [True]* len(actions)
        #set i_th agent action as actions[i] if enable_list[i] is True

    def pause(self):
        #pause the simulation for simulator
        #stop all agent for real world
        pass

    def go_on(self):
        #go on the simulation
        #start all agent for real world
        pass

    def close(self):
        # Handling garbage
        pass