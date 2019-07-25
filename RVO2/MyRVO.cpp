#include "MyRVOSim.h"
#include "Vector2.h"
#include <vector>

/**
void AgentNewVelocity(std::vector<RVO::Vector2> &position, std::vector<RVO::Vector2>& velocity,
                    std::vector<RVO::Vector2> &preferVelocity, const float radius, 
                               const float maxSpeed,   const float timeHorizon, const float neighborDist){
    RVO::RVOSimulator *sim = new RVO::RVOSimulator();
    std::vector<RVO::Vector2> newvelocity;
    for (size_t i = 0; i < position.size(); ++i) {
        sim->addAgent(position[i], velocity[i], preferVelocity[i],radius, maxSpeed, timeHorizon, neighborDist);
    }                               
    sim->doStep();
    for (size_t i = 0; i < position.size(); ++i) {
        newvelocity.push_back(sim->getAgentVelocity(i));
    }
    velocity = newvelocity;
    delete sim;

}***/
/**
void AgentNewVelocity(const std::vector<std::vector<float> > position, std::vector<std::vector<float> > &velocity,
                     const std::vector<std::vector<float> > preferVelocity, const float radius,
                      const float maxSpeed, const float timeHorizon, const float neighborDist, const float timeStep)
{
    RVO::RVOSimulator *sim = new RVO::RVOSimulator();
    std::vector<std::vector<float> > newvelocity;
    newvelocity = velocity;
    sim->setStep(timeStep);
    // Set the agents squence of simulator.
    for (size_t i = 0; i < position.size(); ++i){
        sim->addAgent(position[i][0], position[i][1], velocity[i][0], velocity[i][1], preferVelocity[i][0], 
                      preferVelocity[i][1], radius, maxSpeed, timeHorizon, neighborDist);
    }
    // Compute new velocity.
    sim->doStep();
    // Gets new velocity of each agent.
    for (size_t i = 0; i < position.size(); ++i) {
        sim->getAgentVelocity(i, newvelocity[i]);
    }
    // Update velocity of agents.
    velocity = newvelocity;
    delete sim;
}**/

void AgentNewVel(const int Agentnum, const float pos_x[], const float pos_y[], float velx[], float vely[], const float preVx[], const float preVy[],
                 const float radius[], const float maxSpeed[], const float timeHorizon, const float neighborDist, const float timeStep) 
{
    RVO::RVOSimulator *sim = new RVO::RVOSimulator();
    float *newvelocity;
    sim->setStep(timeStep);
    for (size_t i = 0; i < Agentnum; ++i) {
        sim->addAgent(pos_x[i], pos_y[i], velx[i], vely[i], preVx[i], preVy[i], radius[i], maxSpeed[i], timeHorizon, neighborDist);
    }
    //std::cout<< std::endl<< "newvel";
    sim->doStep();
    for (size_t i = 0; i < Agentnum; i++) {
        newvelocity = sim->getAgentVelocity(i);
        velx[i] = newvelocity[0];
        vely[i] = newvelocity[1];
    }
    delete sim;

}

int Add (int a, int b){
  return a + b;
}
