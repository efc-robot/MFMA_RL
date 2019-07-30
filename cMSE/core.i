%module core
%{
#include"core.h"
%}

#ifndef CORE_H
#define CORE_H
%include "std_vector.i"
namespace std {
  %template(FloatVector) vector<float>;
}
using namespace std;

struct AgentState
{
    float x;
    float y;
    float vel_b;
    float theta;
    float phi;
    bool movable;
    bool crash;
    bool reach;
    float target_x;
    float target_y;
    AgentState();
};

struct Action
{
    float ctrl_vel;
    float ctrl_phi;
    Action();
};

struct Observation
{
    float pos_x;
    float pos_y;
    float pos_theta;
    float pos_target_x;
    float pos_target_y;
    vector<float>laser_data;   //float*n
    Observation();
};

class Agent
{
    public:
        Agent();
        Agent(float,float,float,float,float,float,int,float,float,float,float,float,float,float,bool,float,float);
        void reset(AgentState);
        void reset();
        bool check_AA_collisions(Agent);
        bool check_reach();
        vector<float> laser_agent_agent(Agent);
        float R_safe;
        float R_reach;
        float L_car;
        float W_car;
        float L_axis;
        float R_laser;
        int N_laser;
        float K_vel;
        double K_phi;
        float init_x;
        float init_y;
        float init_theta;
        float init_vel_b;
        float init_phi;
        bool init_movable;
        float init_target_x;
        float init_target_y;
        AgentState state;
        Action action;
        vector<float>laser_state;
};

class World
{
    public:
        World(int, float);
        void SetWorld(int,float,float,float,float,float,float,int,float,float,float,float,float,float,float,bool,float,float);
        void set_action(int,bool,float,float);
        void set_state(int,bool,float,float,float,float,float,bool,bool,bool,float,float);
        AgentState get_state(int);
        Observation get_obs(int);
        Agent get_agent(int);
        void step();
        void apply_action();
        void update_laser_state();
        void integrate_state();
        void check_collisions();
        void check_reach();
        float total_time;
        float dt;
    private:
        Agent *agents;
        int cam_range;
        int num;
};
#endif 
