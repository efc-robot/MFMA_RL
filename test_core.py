from MSE import core
from scenario.paser import  parse_senario
scenario = parse_senario('./scenario/scenario.yaml')
world = core.World(scenario['agent_groups'])
world.reset()
world._reset_render()
state = world.get_state()


for i in range(1000):
    world.step()
    action = core.Action()
    action.ctrl_vel = 0.5
    action.ctrl_phi = 0.22
    world.set_action([True] * 4,[action,action,action,action])
    obs = world.get_obs()
    print(obs)
    #print('****************')
    world.render()
