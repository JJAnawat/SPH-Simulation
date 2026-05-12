#include "SPHSolver.h"
#include "simulation/SmoothKernel.h"
#include "simulation/SimParams.h"
#include "simulation/ParticleUtils.h"

#include <algorithm>
#include <cmath>

void SPHSolver::updateGrid(){
    grid.clear();
    auto pos_vec = extractPos(particles);
    grid.insert(pos_vec);
}

void SPHSolver::computeDensityPressure() {
    const float h2 = Sim::params.h * Sim::params.h;

    const int particleCount = static_cast<int>(particles.size());

#ifdef _OPENMP
#pragma omp parallel
    {
        std::vector<int> neighbors;
        neighbors.reserve(128);

#pragma omp for schedule(static)
        for (int i = 0; i < particleCount; ++i) {
            Particle& pi = particles[i];
            pi.density = Sim::params.p_mass * w_poly6(0.f); // self contribution

            grid.query(pi.position, neighbors);
            for (int j : neighbors) {
                if (i == j) continue;
                Particle& pj = particles[j];

                glm::vec3 r = pj.position - pi.position;
                const float r2 = glm::dot(r, r);
                if (r2 > h2)
                    continue;

                // pi.density += pj.mass * w_poly6(r);
                pi.density += Sim::params.p_mass * w_poly6(r2);
            }

            pi.pressure = std::max(0.f, Sim::params.k * (pi.density - Sim::params.rho0));
        }
#pragma omp barrier
    }
#else
    std::vector<int> neighbors;
    neighbors.reserve(128);

    for (int i = 0; i < particleCount; ++i) {
        Particle& pi = particles[i];
        pi.density = Sim::params.p_mass * w_poly6(0.f); // self contribution

        grid.query(pi.position, neighbors);
        for (int j : neighbors) {
            if (i == j) continue;
            Particle& pj = particles[j];

            glm::vec3 r = pj.position - pi.position;
            const float r2 = glm::dot(r, r);
            if (r2 > h2)
                continue;

            pi.density += Sim::params.p_mass * w_poly6(r2);
        }

        pi.pressure = std::max(0.f, Sim::params.k * (pi.density - Sim::params.rho0));
    }
#endif
}

void SPHSolver::computeForces(){
    const int particleCount = static_cast<int>(particles.size());

#ifdef _OPENMP
#pragma omp parallel
    {
        std::vector<int> neighbors;
        neighbors.reserve(128);

#pragma omp for schedule(static)
        for (int i = 0; i < particleCount; ++i) {
            Particle& pi = particles[i];
            glm::vec3 f_press(0.f);
            glm::vec3 f_visc(0.f);

            if (pi.density < 1e-6f) {
                pi.acceleration = glm::vec3(0.f, Sim::params.gravity, 0.f);
                continue;
            }

            grid.query(pi.position, neighbors);
            for (int j : neighbors) {
                Particle& pj = particles[j];
                if (&pi == &pj) continue;

                glm::vec3 r = pj.position - pi.position;
                float r_len = glm::length(r);
                if (r_len < 1e-6f || r_len > Sim::params.h)
                    continue;

                const glm::vec3 direction = r / r_len;
                f_press += direction * Sim::params.p_mass * ((pi.pressure + pj.pressure) / (2 * pj.density)) * grad_w_spiky(r_len);
                f_visc += Sim::params.mu * Sim::params.p_mass * (pj.velocity - pi.velocity) / pj.density * laplacian_w_viscosity(r_len);
            }

            pi.acceleration = (f_press + f_visc) / pi.density + glm::vec3(0.f, Sim::params.gravity, 0.f); // Pressure + Viscosity + Gravity
        }
#pragma omp barrier
    }
#else
    std::vector<int> neighbors;
    neighbors.reserve(128);

    for (auto& pi : particles) {
        glm::vec3 f_press(0.f);
        glm::vec3 f_visc(0.f);

        if (pi.density < 1e-6f) {
            pi.acceleration = glm::vec3(0.f, Sim::params.gravity, 0.f);
            continue;
        }

        grid.query(pi.position, neighbors);
        for (int j : neighbors) {
            Particle& pj = particles[j];
            if (&pi == &pj) continue;

            glm::vec3 r = pj.position - pi.position;
            float r_len = glm::length(r);
            if (r_len < 1e-6f || r_len > Sim::params.h)
                continue;

            const glm::vec3 direction = r / r_len;
            f_press += direction * Sim::params.p_mass * ((pi.pressure + pj.pressure) / (2 * pj.density)) * grad_w_spiky(r_len);
            f_visc += Sim::params.mu * Sim::params.p_mass * (pj.velocity - pi.velocity) / pj.density * laplacian_w_viscosity(r_len);
        }

        pi.acceleration = (f_press + f_visc) / pi.density + glm::vec3(0.f, Sim::params.gravity, 0.f); // Pressure + Viscosity + Gravity
    }
#endif
}

void SPHSolver::applyRigidBoundaryCoupling(std::vector<RigidBody>& rigidBodies, bool accumulateToRigid) {
    if (rigidBodies.empty()) return;

    const float h = Sim::params.h;
    const float h2 = h * h;
    const float eps = 1e-6f;
    std::vector<int> nearbyParticles;
    nearbyParticles.reserve(128);

    for (auto& body : rigidBodies) {
        if (accumulateToRigid) body.clearAccumulators();

        const auto& samples = body.worldSamples();
        for (const auto& sampleWorldPos : samples) {
            const glm::vec3 sampleVelocity =
                body.linearVelocity() + glm::cross(body.angularVelocity(), sampleWorldPos - body.position());

            grid.query(sampleWorldPos, nearbyParticles);
            for (int pIdx : nearbyParticles) {
                Particle& p = particles[pIdx];
                const glm::vec3 d = p.position - sampleWorldPos;
                const float dist2 = glm::dot(d, d);
                if (dist2 >= h2 || dist2 <= eps) {
                    continue;
                }

                const float dist = std::sqrt(dist2);
                const glm::vec3 normal = d / dist;
                const float penetration = h - dist;

                const glm::vec3 relVelocity = p.velocity - sampleVelocity;
                const float normalSpeed = glm::dot(relVelocity, normal);
                const float dampingTerm = std::max(0.f, -normalSpeed) * Sim::params.rigidBoundaryDamping;
                const float stiffnessTerm = Sim::params.rigidBoundaryStiffness * (penetration / h);
                
                //now I cap accelTerm at 25
                const float accelTerm = std::min(stiffnessTerm + dampingTerm, 25.f); // acceleration to add to particle
                p.acceleration += normal * accelTerm;

                if (accumulateToRigid) {
                    const glm::vec3 forceOnParticle = p.mass * normal * accelTerm;
                    const glm::vec3 forceOnRigid = -forceOnParticle;
                    body.applyForceAtPoint(forceOnRigid, sampleWorldPos);
                }
            }
        }
    }
}

void SPHSolver::integrateFluid(float dt) {
    for (auto &p : particles) {
        p.velocity += p.acceleration * dt;
        p.position += p.velocity * dt;

        //temporary bug fix clamp velocity to be less than 100
        if (glm::length(p.velocity) > 10.f) {
            p.velocity = glm::normalize(p.velocity) * 10.f;
        }
        

        // Catch NaN
        if (glm::any(glm::isnan(p.position))) {
            printf("NaN detected! density=%.4f acc=(%.2f,%.2f,%.2f)\n",
                p.density, p.acceleration.x, p.acceleration.y, p.acceleration.z);
            p.position = glm::vec3(0.f);
            p.velocity = glm::vec3(0.f);
            break;
        }
    }
}

void SPHSolver::step(float dt){
    std::vector<RigidBody> emptyBodies;
    step(dt, emptyBodies);
}

void SPHSolver::step(float dt, std::vector<RigidBody>& rigidBodies) {
    computeDensityPressure();
    computeForces();

    // If two-way coupling enabled, accumulate forces/torques on rigid bodies then integrate them
    if (Sim::params.rigidTwoWayCoupling) {
        // Compute interactions and accumulate on rigid bodies
        applyRigidBoundaryCoupling(rigidBodies, true);

        // Integrate rigid bodies using accumulated forces and gravity
        for (auto& body : rigidBodies) {
            body.applyForce({0.f, body.mass() * Sim::params.gravity, 0.f});
            body.integrate(dt);
        }

        // After rigid moved, apply boundary response to fluid particles using updated samples
        applyRigidBoundaryCoupling(rigidBodies, false);
    }
    else if (Sim::params.rigidOneWayCoupling) {
        // simple one-way: apply particle acceleration due to rigid samples (no forces to rigid)
        applyRigidBoundaryCoupling(rigidBodies, false);
    }

    // When two-way coupling is disabled we still need to integrate rigid bodies (gravity, etc.).
    if (!rigidBodies.empty() && !Sim::params.rigidTwoWayCoupling) {
        for (auto& body : rigidBodies) {
            body.applyForce({0.f, body.mass() * Sim::params.gravity, 0.f});
            body.integrate(dt);
        }
    }

    integrateFluid(dt);
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
