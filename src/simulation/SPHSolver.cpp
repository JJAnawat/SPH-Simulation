#include "SPHSolver.h"
#include "simulation/SimParams.h"
#include <algorithm>

SPHSolver::SPHSolver(const float h, std::vector<Particle>& parts) : grid(h), particles(parts) {
    std::vector<glm::vec3> pos_vec;
    std::transform(particles.begin(), particles.end(), std::back_inserter(pos_vec),
                    [](const Particle& p) {return p.position; });
    grid.insert(pos_vec);
}

void SPHSolver::step(float dt){
    for(auto &p : particles){
        p.acceleration = glm::vec3(0.f, Sim::params.gravity ,0.f);
        
        p.velocity += p.acceleration * dt;
        p.position += p.velocity * dt;

        handleBoundaries();
    }
}

void SPHSolver::handleBoundaries(){
    for(auto &p : particles){
        for(int ax=0; ax<3; ax++){
            if(p.position[ax] < Sim::params.boxMin[ax]){
                p.position[ax] = Sim::params.boxMin[ax] + 0.001f;
                if(p.velocity[ax] < 0.0f)
                    p.velocity[ax] *= -Sim::params.damp;
            }
            if(p.position[ax] > Sim::params.boxMax[ax]){
                p.position[ax] = Sim::params.boxMax[ax] - 0.001f;
                if(p.velocity[ax] > 0.0f)
                    p.velocity[ax] *= -Sim::params.damp;
            }
        }
    }
}