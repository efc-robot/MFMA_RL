import ctypes
so = ctypes.CDLL('./build/src/libRVO.so')
import math
import numpy as np

def setpreferv(pos_x, pos_y, goal_x, goal_y, pre_vel):
    x_relative = goal_x - pos_x
    y_relative = goal_y - pos_y
    distance = math.sqrt(x_relative * x_relative + y_relative * y_relative)
    if distance > 1.0:
        pre_vel[0] = x_relative / distance
        pre_vel[1] = y_relative / distance
    else:
        pre_velx = x_relative
        pre_vely = y_relative


def reach_goal(Agent_num, Pos_x, Pos_y, Goal_x, Goal_y, Radius):
    for i in range(0, Agent_num):
        x_relative = Goal_x[i] - Pos_x[i]
        y_relative = Goal_y[i] - Pos_y[i]
        distance = x_relative * x_relative + y_relative * y_relative
        if distance > Radius * Radius :
            return False
    return True


agent_num = 100
agent_num_ = 100.0
coordinate = 200.0
globaltime = 0.0

vel_temp = [0.0, 0.0]
goal_x = []
goal_y = []
Radius = 1.5

flag = False

x_position = []
y_position = []
velocity_x = []
velocity_y = []
prefer_vel_x = []
prefer_vel_y = []
radius = []
maxspeed = []
time_Horizon = ctypes.c_float(10.0)
neighbor_Dist = ctypes.c_float(15.0)
timestep = ctypes.c_float(0.5)
time_step = 0.5

for i in range(0, agent_num):
    x_position.append(coordinate * math.cos(i * 2.0 * math.pi / agent_num_))
    y_position.append(coordinate * math.sin(i * 2.0 * math.pi / agent_num_))
    goal_x.append(-x_position[i])
    goal_y.append(-y_position[i])
    setpreferv(x_position[i], y_position[i], goal_x[i], goal_y[i], pre_vel =  vel_temp)
    velocity_x.append(vel_temp[0])
    velocity_y.append(vel_temp[1])
    prefer_vel_x.append(vel_temp[0])
    prefer_vel_y.append(vel_temp[1])
    radius.append(1.5)
    maxspeed.append(1.5)

while not flag:
    print(globaltime, end = "")
    for i in range(0, agent_num):
        print(" (%f,%f)"%(x_position[i],y_position[i]),end = "")
    print('\n', end="")
    cpp_x_position = (ctypes.c_float * agent_num)(*x_position)
    cpp_y_position = (ctypes.c_float * agent_num)(*y_position)
    cpp_velocity_x = (ctypes.c_float * agent_num)(*velocity_x)
    cpp_velocity_y = (ctypes.c_float * agent_num)(*velocity_y)
    cpp_prefer_vel_x = (ctypes.c_float * agent_num)(*prefer_vel_x)
    cpp_prefer_vel_y = (ctypes.c_float * agent_num)(*prefer_vel_y)
    cpp_radius = (ctypes.c_float * agent_num)(*radius)
    cpp_maxspeed = (ctypes.c_float * agent_num)(*maxspeed) 
    so._Z11AgentNewVeliPKfS0_PfS1_S0_S0_S0_S0_fff( agent_num, cpp_x_position, cpp_y_position, cpp_velocity_x, cpp_velocity_y, cpp_prefer_vel_x, cpp_prefer_vel_y, cpp_radius, cpp_maxspeed, time_Horizon, neighbor_Dist, timestep)
    temp_velocity_x = np.array(cpp_velocity_x)
    temp_velocity_y = np.array(cpp_velocity_y)
    velocity_x = temp_velocity_x.tolist()
    velocity_y = temp_velocity_y.tolist()

    for i in range(0, agent_num):
        x_position[i] = x_position[i] + velocity_x[i] * time_step;
        y_position[i] = y_position[i] + velocity_y[i] * time_step;
        setpreferv(x_position[i], y_position[i], goal_x[i], goal_y[i], pre_vel =  vel_temp)
        prefer_vel_x[i] = vel_temp[0]
        prefer_vel_y[i] = vel_temp[1]
    
    globaltime += time_step

    flag = reach_goal(Agent_num = agent_num, Pos_x = x_position, Pos_y = y_position, Goal_x = goal_x, Goal_y = goal_y, Radius = Radius)
    if flag == True:
        print(globaltime, end = "")
        for i in range(0, agent_num):
            print(" (%f,%f)"%(x_position[i],y_position[i]),end = "")
        print('\n', end="")   
