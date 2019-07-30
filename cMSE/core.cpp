#include"core.h"

AgentState::AgentState()
{
    x = 0;
    y = 0;
    vel_b = 0;
    theta = 0;
    phi = 0;
    movable = true;
    crash = false;
    reach = false;
    target_x = 1;
    target_y = 1;
};

Action::Action()
{
    ctrl_vel = 0;
    ctrl_phi = 0;
};

Observation::Observation()
{
    pos_x = 0;
    pos_y = 0;
    pos_theta = 0;
    pos_target_x = 0;
    pos_target_y = 0;
};

Agent::Agent()
{
    R_safe = 0.2;
    R_reach = 0.1;
    L_car = 0.3;
    W_car = 0.2;
    L_axis = 0.25;
    R_laser = 4;
    N_laser = 360;
    K_vel = 1;
    K_phi = 30;
    init_x = -1;
    init_y = -1;
    init_theta = 0;
    init_vel_b = 0;
    init_phi = 0;
    init_movable = true;
    init_target_x = 1;
    init_target_y = 1;
    AgentState state = AgentState();
    Action action = Action();
    for(int i=0;i<N_laser;i++) laser_state.push_back(R_laser);
    reset();
};

Agent::Agent(float R_safe,float R_reach,float L_car,float W_car,float L_axis,float R_laser,int N_laser,float K_vel,
float K_phi,float init_x,float init_y,float init_theta,float init_vel_b,float init_phi,bool init_movable,
float init_target_x,float init_target_y)
{
    this->R_safe = R_safe;
    this->R_reach = R_reach;
    this->L_car = L_car;
    this->W_car = W_car;
    this->L_axis = L_axis;
    this->R_laser = R_laser;
    this->N_laser = N_laser;
    this->K_vel = K_vel;
    this->K_phi = K_phi;
    this->init_x = init_x;
    this->init_y = init_y;
    this->init_theta = init_theta;
    this->init_vel_b = init_vel_b;
    this->init_phi = init_phi;
    this->init_movable = init_movable;
    this->init_target_x = init_target_x;
    this->init_target_y = init_target_y;
    AgentState state = AgentState();
    Action action = Action();
    for(int i=0;i<N_laser;i++) laser_state.push_back(R_laser);
    reset();
};

void Agent::reset(AgentState s)
{
    state = s;
    for(int i=0;i<N_laser;i++) laser_state[i] = R_laser;
};

void Agent::reset()
{
    state.x = init_x;
    state.y = init_y;
    state.theta = init_theta;
    state.vel_b = init_vel_b;
    state.phi = init_phi;
    state.movable = init_movable;
    state.crash = false;
    state.reach = false;
    for(int i=0;i<N_laser;i++) laser_state[i] = R_laser;
};

bool Agent::check_AA_collisions(Agent agent_b)
{
    float min_dist = pow((R_safe + agent_b.R_safe),2);
    float ab_dist = pow((state.x - agent_b.state.x),2) + pow((state.y - agent_b.state.y),2);
    return ab_dist <= min_dist;
};

bool Agent::check_reach()
{
    float max_dist = pow(R_reach,2);
    float at_dist = pow((state.x - state.target_x),2) + pow((state.y - state.target_y),2);
    return at_dist<=max_dist;
};

vector<float> Agent::laser_agent_agent(Agent agent_b)
{
    float R = R_laser;
    int N = N_laser;
    vector<float>l_laser;
    for(int i=0;i<N;i++) l_laser.push_back(R);
    float o_pos[2] = {state.x,state.y};
    float oi_pos[2] = {agent_b.state.x,agent_b.state.y};
    float l1 = sqrt((o_pos[0]-oi_pos[0])*(o_pos[0]-oi_pos[0])+(o_pos[1]-oi_pos[1])*(o_pos[1]-oi_pos[1]));
    float l2 = R + sqrt(agent_b.L_car*agent_b.L_car + agent_b.W_car*agent_b.W_car)/2.0;
    if(l1>l2) return l_laser;
    float theta = this->state.theta;
    float theta_b = agent_b.state.theta;
    float cthb = cos(theta_b);
    float sthb = sin(theta_b);
    float half_l_shift[2] = {cthb*agent_b.L_car/2.0,sthb*agent_b.L_car/2.0}; 
    float half_w_shift[2] = {-sthb*agent_b.W_car/2.0,cthb*agent_b.W_car/2.0};
    float car_point[4][2];
    car_point[0][0] = oi_pos[0]+half_l_shift[0]+half_w_shift[0]-o_pos[0];
    car_point[0][1] = oi_pos[1]+half_l_shift[1]+half_w_shift[1]-o_pos[1];
    car_point[1][0] = oi_pos[0]-half_l_shift[0]+half_w_shift[0]-o_pos[0];
    car_point[1][1] = oi_pos[1]-half_l_shift[1]+half_w_shift[1]-o_pos[1];
    car_point[2][0] = oi_pos[0]-half_l_shift[0]-half_w_shift[0]-o_pos[0];
    car_point[2][1] = oi_pos[1]-half_l_shift[1]-half_w_shift[1]-o_pos[1];
    car_point[3][0] = oi_pos[0]+half_l_shift[0]-half_w_shift[0]-o_pos[0];
    car_point[3][1] = oi_pos[1]+half_l_shift[1]-half_w_shift[1]-o_pos[1];
    for(int i=0;i<4;i++)
    {
        float start_point[2] = {car_point[i][0],car_point[i][1]};
        float end_point[2] = {car_point[(i+1)%4][0],car_point[(i+1)%4][1]};
        float tao_es[2] = {start_point[1]-end_point[1],end_point[0]-start_point[0]};
        tao_es[0] = tao_es[0]/sqrt(tao_es[0]*tao_es[0]*+tao_es[1]*tao_es[1]);
        tao_es[1] = tao_es[1]/sqrt(tao_es[0]*tao_es[0]*+tao_es[1]*tao_es[1]);
        if(fabs(start_point[0]*tao_es[0]+start_point[1]*tao_es[1])>R) continue;
        if((start_point[0]*end_point[1]-start_point[1]*end_point[0])<0)
        {
            float temp;
            temp = start_point[0];
            start_point[0] = end_point[0];
            end_point[0] = temp;
            temp = start_point[1];
            start_point[1] = end_point[1];
            end_point[1] = temp;
        }
        float theta_start = acos(start_point[0]/sqrt(start_point[0]*start_point[0]+start_point[1]*start_point[1]));
        if(start_point[1]<0) theta_start = 2*pi-theta_start;
        theta_start -= theta;
        float theta_end = acos(end_point[0]/sqrt(end_point[0]*end_point[0]+end_point[1]*end_point[1]));
        if(end_point[1]<0) theta_end = 2*pi-theta_end;
        theta_end -= theta;
        float laser_idx_start = theta_start/(2*pi/N);
        float laser_idx_end = theta_end/(2*pi/N);
        if(laser_idx_start > laser_idx_end) laser_idx_end += N;
        if(floor(laser_idx_end)-floor(laser_idx_start)==0) continue;
        laser_idx_start = ceil(laser_idx_start);
        laser_idx_end = ceil(laser_idx_end);
        for(int laser_idx=laser_idx_start;laser_idx<laser_idx_end+1;laser_idx++)
        {
            int laser_idx_ = laser_idx%N;
            float x1 = start_point[0];
            float y1 = start_point[1];
            float x2 = end_point[0];
            float y2 = end_point[1];
            float theta_i = theta+laser_idx_*2*pi/N;
            float cthi = cos(theta_i);
            float sthi = sin(theta_i);
            float temp = (y1-y2)*cthi-(x1-x2)*sthi;
            float dist;
            if(fabs(temp)<=1e-10) dist = R;
            else dist = (x2*y1-x1*y2)/temp;
            if(dist>0) l_laser[laser_idx] = min(l_laser[laser_idx],dist);
        }
    }
    return l_laser;
};

World::World(int num,float cfg = 0.1)
{
    this->num = num;
    agents = new Agent[num];
    dt = cfg;
    cam_range = 4;
    total_time = 0;
};

void World::SetWorld(int index,float R_safe,float R_reach,float L_car,float W_car,float L_axis,float R_laser,int N_laser,float K_vel,
float K_phi,float init_x,float init_y,float init_theta,float init_vel_b,float init_phi,bool init_movable,
float init_target_x,float init_target_y)
{
    agents[index].R_safe = R_safe;
    agents[index].R_reach = R_reach;
    agents[index].L_car = L_car;
    agents[index].W_car = W_car;
    agents[index].L_axis = L_axis;
    agents[index].R_laser = R_laser;
    agents[index].N_laser = N_laser;
    agents[index].K_vel = K_vel;
    agents[index].K_phi = K_phi;
    agents[index].init_x = init_x;
    agents[index].init_y = init_y;
    agents[index].init_theta = init_theta;
    agents[index].init_vel_b = init_vel_b;
    agents[index].init_phi = init_phi;
    agents[index].init_movable = init_movable;
    agents[index].init_target_x = init_target_x;
    agents[index].init_target_y = init_target_y;
};

void World::set_action(int action_idx,bool enable,float vel,float phi)
{
    if(enable == true)
    {
        agents[action_idx].action.ctrl_vel = vel;
        agents[action_idx].action.ctrl_phi = phi;
    }
};

void World::set_state(int state_idx,bool enable,float x,float y,float vel_b,float theta,float phi,bool movable,
bool crash,bool reach,float target_x,float target_y)
{
    if(enable == true)
    {
        agents[state_idx].state.x = x;
        agents[state_idx].state.y = y;
        agents[state_idx].state.vel_b = vel_b;
        agents[state_idx].state.theta = theta;
        agents[state_idx].state.phi = phi;
        agents[state_idx].state.movable = movable;
        agents[state_idx].state.crash = crash;
        agents[state_idx].state.reach = reach;
        agents[state_idx].state.target_x = target_x;
        agents[state_idx].state.target_y = target_y;
    }
};

AgentState World::get_state(int gstate_idx)
{
    return agents[gstate_idx].state;
};

Observation World::get_obs(int obs_idx)
{
    if(obs_idx==0) update_laser_state();
    AgentState state = agents[obs_idx].state;
    Observation obs;
    obs.pos_x = state.x;
    obs.pos_y = state.y;
    obs.pos_theta = state.theta;
    obs.pos_target_x = state.target_x;
    obs.pos_target_y = state.target_y;
    obs.laser_data = agents[obs_idx].laser_state;
    return obs;
};

Agent World::get_agent(int agent_idx)
{
    return agents[agent_idx];
}

void World::step()
{
    apply_action();
    integrate_state();
    check_collisions();
    check_reach();
    total_time += dt;
};

void World::apply_action()
{
    for(int i=0;i<num;i++)
    {
        if(agents[i].state.movable)
        {
            if(agents[i].action.ctrl_vel>1.0) agents[i].state.vel_b = 1.0*agents[i].K_vel;
            else if(agents[i].action.ctrl_vel<-1.0) agents[i].state.vel_b = -1.0*agents[i].K_vel;
            else agents[i].state.vel_b = agents[i].action.ctrl_vel*agents[i].K_vel;
            if(agents[i].action.ctrl_phi>1.0) agents[i].state.phi = 1.0*agents[i].K_phi;
            else if(agents[i].action.ctrl_phi<-1.0) agents[i].state.phi = -1.0*agents[i].K_phi;
            else agents[i].state.phi = agents[i].action.ctrl_phi*agents[i].K_phi;
        }
    }
};

void World::update_laser_state()
{
    for(int idx_a=0;idx_a<num;idx_a++)
    {
        for(int i=0;i<agents[idx_a].N_laser;i++) agents[idx_a].laser_state[i] = agents[idx_a].R_laser; 
        for(int idx_b=0;idx_b<num;idx_b++)
        {
            if(idx_a==idx_b) continue;
            vector<float>l_laser = agents[idx_a].laser_agent_agent(agents[idx_b]);
            for(int j=0;j<agents[idx_a].N_laser;j++) agents[idx_a].laser_state[j] = min(agents[idx_a].laser_state[j],l_laser[j]);
        }
    }
};

void World::integrate_state()
{
    for(int i=0;i<num;i++)
    {
        if(agents[i].state.movable==false) continue;
        float _phi = agents[i].state.phi;
        float _vb = agents[i].state.vel_b;
        float _theta = agents[i].state.theta;
        float sth = sin(_theta);
        float cth = cos(_theta);
        float _L = agents[i].L_axis;
        float _xb = agents[i].state.x-cth*_L/2.0;
        float _yb = agents[i].state.y-sth*_L/2.0;
        float tphi = tan(_phi);
        float _omega = _vb/_L*tphi;
        float _delta_theta = _omega*dt;
        float _rb,_delta_tao,_delta_yeta;
        if(fabs(_phi)>0.00001)
        {
            _rb = _L/tphi;
            _delta_tao = _rb*(1-cos(_delta_theta));
            _delta_yeta = _rb*sin(_delta_theta);
        }
        else
        {
            _delta_tao = _vb*dt*(_delta_theta/2.0);
            _delta_yeta = _vb*dt*(1-_delta_theta*_delta_theta/6.0);
        }
        _xb += _delta_yeta*cth - _delta_tao*sth;
        _yb += _delta_yeta*sth + _delta_tao*cth;
        _theta += _delta_theta;
        float fdec = (_theta/pi) - (int)(_theta/pi);
        _theta = (int)(_theta/pi)%2*pi+fdec*pi;
        agents[i].state.x = _xb+cos(_theta)*_L/2.0;
        agents[i].state.y = _yb+sin(_theta)*_L/2.0;
        agents[i].state.theta = _theta;
    }
};

void World::check_collisions()
{
    for(int ia=0;ia<num;ia++)
    {
        if(agents[ia].state.crash) continue;
        for(int ib=0;ib<num;ib++)
        {
            if(ia==ib) continue;
            if(agents[ia].check_AA_collisions(agents[ib]))
            {
                agents[ia].state.crash = true;
                agents[ia].state.movable = false;
                break;
            }
        }
    }
};

void World::check_reach()
{
    for(int i=0;i<num;i++)
    {
        bool reach = agents[i].check_reach();
        if(reach==true)
        {
            agents[i].state.reach = true;
            agents[i].state.movable = false;
        }
    }
};