#include "SPHSolver.h"
#include "simulation/SmoothKernel.h"
#include "simulation/SimParams.h"
#include "simulation/ParticleUtils.h"

void SPHSolver::updateGrid(){
    grid.clear();
    auto pos_vec = extractPos(particles);
    grid.insert(pos_vec);
}

void SPHSolver::computeDensityPressure() {
    std::vector<Particle> n_particles = particles;
    for(int i=0;i<n_particles.size();i++){
        Particle &pi = n_particles[i];
        pi.density = pi.mass * w_poly6(glm::vec3(0.f));

        auto neighbors = grid.query(pi.position);
        for(int j : neighbors){
            if(i==j)
                continue;

            Particle &pj = particles[j];
            
            glm::vec3 r = pj.position - pi.position;
            if(glm::length(r) > Sim::params.h)
                continue;

            pi.density += pj.mass * w_poly6(r);
        }

        pi.pressure = std::max(0.f, Sim::params.k * (pi.density - Sim::params.rho0)); 
    }

    particles = n_particles;
}

void SPHSolver::computeForces(){
    for(auto &pi : particles){
        glm::vec3 f_press(0.f);
        glm::vec3 f_visc(0.f);

        if(pi.density < 1e-6){
            pi.acceleration = glm::vec3(0.f, Sim::params.gravity, 0.f);
            continue;
        }

        auto neighbors = grid.query(pi.position);
        for(int j : neighbors){
            Particle &pj = particles[j];
        
            glm::vec3 r = pj.position - pi.position;
            float r_len = glm::length(r);
            if(r_len < 1e-6 || r_len > Sim::params.h)
                continue;

            f_press += -glm::normalize(r) * pj.mass * ((pi.pressure + pj.pressure) / (2 * pj.density)) * grad_w_spiky(r);
            f_visc += Sim::params.mu * pj.mass * (pj.velocity - pi.velocity) / pj.density * laplacian_w_viscosity(r);
        }

        pi.acceleration = (f_press + f_visc) / pi.density + glm::vec3(0.f, Sim::params.gravity, 0.f); // Pressure + Viscosity + Gravity
    }
}

void SPHSolver::step(float dt){
    computeDensityPressure();
    computeForces();
    // printf("particle[0] density=%.4f pressure=%.4f\n",
    //    particles[0].density, particles[0].pressure);
    for(auto &p : particles){
        p.velocity += p.acceleration * dt;
        p.position += p.velocity * dt;

        // Catch NaN
        if (glm::any(glm::isnan(p.position))) {
            printf("NaN detected! density=%.4f acc=(%.2f,%.2f,%.2f)\n",
                p.density, p.acceleration.x, p.acceleration.y, p.acceleration.z);
            p.position = glm::vec3(0.f);
            p.velocity = glm::vec3(0.f);
            break;
        }
    }
    handleBoundaries();
    updateGrid();
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