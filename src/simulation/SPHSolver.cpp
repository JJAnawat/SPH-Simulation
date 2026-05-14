#include "SPHSolver.h"
#include "simulation/SmoothKernel.h"
#include "simulation/SimParams.h"
#include "simulation/ParticleUtils.h"

#include <algorithm>
#include <cmath>
#include <omp.h>

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
                pi.density = Sim::params.p_mass * w_poly6(glm::vec3(0.f)); // self contribution

                grid.query(pi.position, neighbors);
                for (int j : neighbors) {
                    if (i == j) continue;
                    Particle& pj = particles[j];

                    glm::vec3 r = pj.position - pi.position;
                    // Use squared distance to avoid sqrt
                    if(glm::dot(r, r) > h2)
                        continue;

                    pi.density += Sim::params.p_mass * w_poly6(r);
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
            pi.density = Sim::params.p_mass * w_poly6(glm::vec3(0.f)); // self contribution

            grid.query(pi.position, neighbors);
            for (int j : neighbors) {
                if (i == j) continue;
                Particle& pj = particles[j];

                glm::vec3 r = pj.position - pi.position;
                if(glm::dot(r, r) > h2)
                    continue;

                pi.density += Sim::params.p_mass * w_poly6(r);
            }

            pi.pressure = std::max(0.f, Sim::params.k * (pi.density - Sim::params.rho0));
        }
    #endif
    }

void SPHSolver::computeForces(){
    const float h = Sim::params.h;
    const float h2 = h * h;
    const float eps = 1e-6f;
        const glm::vec3 gravity = Sim::params.gravityVector();
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
                pi.acceleration = gravity;
                continue;
            }

            grid.query(pi.position, neighbors);
            for (int j : neighbors) {
                Particle& pj = particles[j];
                if (&pi == &pj) continue;

                glm::vec3 r = pj.position - pi.position;
                float r_len2 = glm::dot(r, r);
                // Use squared distance to avoid sqrt
                if (r_len2 < eps * eps || r_len2 > h2)
                    continue;

                float r_len = std::sqrt(r_len2);
                glm::vec3 r_norm = r / r_len;  // normalize once and reuse
                f_press += r_norm * Sim::params.p_mass * ((pi.pressure + pj.pressure) / (2 * pj.density)) * grad_w_spiky(r);
                f_visc += Sim::params.mu * Sim::params.p_mass * (pj.velocity - pi.velocity) / pj.density * laplacian_w_viscosity(r);
            }

            pi.acceleration = (f_press + f_visc) / pi.density + gravity;
        }
    #pragma omp barrier
        }
    #else
        std::vector<int> neighbors;
        neighbors.reserve(128);

        for (int i = 0; i < particleCount; ++i) {
            Particle& pi = particles[i];
            glm::vec3 f_press(0.f);
            glm::vec3 f_visc(0.f);

            if (pi.density < 1e-6f) {
                pi.acceleration = gravity;
                continue;
            }

            grid.query(pi.position, neighbors);
            for (int j : neighbors) {
                Particle& pj = particles[j];
                if (&pi == &pj) continue;

                glm::vec3 r = pj.position - pi.position;
                float r_len2 = glm::dot(r, r);
                if (r_len2 < eps * eps || r_len2 > h2)
                    continue;

                float r_len = std::sqrt(r_len2);
                glm::vec3 r_norm = r / r_len;
                f_press += r_norm * Sim::params.p_mass * ((pi.pressure + pj.pressure) / (2 * pj.density)) * grad_w_spiky(r);
                f_visc += Sim::params.mu * Sim::params.p_mass * (pj.velocity - pi.velocity) / pj.density * laplacian_w_viscosity(r);
            }

            pi.acceleration = (f_press + f_visc) / pi.density + gravity;
        }
    #endif
    }

void SPHSolver::applyRigidBoundaryCoupling(std::vector<RigidBody>& rigidBodies, bool accumulateToRigid, float dt) {
    if (rigidBodies.empty()) return;

    const float h = Sim::params.h;
    const float h2 = h * h;
    const float eps = 1e-6f;
    const float invRestDensity = Sim::params.rho0 > eps ? 1.f / Sim::params.rho0 : 0.f;
    std::vector<int> nearbyParticles;
    nearbyParticles.reserve(128);

    // maximum allowed velocity change from coupling per substep
    const float maxDv = 5.f;
    const float invDtForClamp = (dt > 1e-8f) ? (1.f / dt) : 1e8f;

    for (auto& body : rigidBodies) {
        if (accumulateToRigid) body.clearAccumulators();

        const auto& samples = body.worldSamples();
        if (samples.empty()) continue;
        const float sampleWeight = 1.f / static_cast<float>(samples.size());

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

                // Denser fluid should push rigid bodies harder, but keep the scale bounded.
                const float densityScale = std::clamp(p.density * invRestDensity, 0.35f, 2.5f);

                const float rawAccel = (stiffnessTerm + dampingTerm) * densityScale;

                // spatial weight (fade with distance) and per-sample normalization
                const float spatialWeight = std::max(0.f, (h - dist) / h);
                float accelMag = rawAccel * spatialWeight * sampleWeight;

                // clamp by max dv per substep to avoid explosive impulses
                const float maxAccel = maxDv * invDtForClamp;
                if (accelMag > maxAccel) accelMag = maxAccel;

                p.acceleration += normal * accelMag;

                if (accumulateToRigid) {
                    const glm::vec3 forceOnParticle = Sim::params.p_mass * normal * accelMag;
                    const glm::vec3 forceOnRigid = -forceOnParticle;
                    body.applyForceAtPoint(forceOnRigid, sampleWorldPos);
                }
            }
        }
    }
}

void SPHSolver::integrateFluid(float dt) {
    const int particleCount = static_cast<int>(particles.size());

    #ifdef _OPENMP
    #pragma omp parallel for schedule(static)
    for (int i = 0; i < particleCount; ++i) {
        Particle &p = particles[i];
        p.velocity += p.acceleration * dt;
        p.position += p.velocity * dt;

        // Catch NaN (silent fix, no printing in inner loop)
        if (glm::any(glm::isnan(p.position))) {
            p.position = glm::vec3(0.f);
            p.velocity = glm::vec3(0.f);
        }
    }
    #else
    
    for (int i = 0; i < particleCount; ++i) {
        Particle &p = particles[i];
        p.velocity += p.acceleration * dt;
        p.position += p.velocity * dt;

        // Catch NaN (silent fix, no printing in inner loop)
        if (glm::any(glm::isnan(p.position))) {
            p.position = glm::vec3(0.f);
            p.velocity = glm::vec3(0.f);
        }
    }
    #endif
}

void SPHSolver::step(float dt){
    std::vector<RigidBody> emptyBodies;
    step(dt, emptyBodies);
}

void SPHSolver::step(float dt, std::vector<RigidBody>& rigidBodies) {
    const glm::vec3 gravity = Sim::params.gravityVector();
    computeDensityPressure();
    computeForces();

    // If two-way coupling enabled, accumulate forces/torques on rigid bodies then integrate them
    if (Sim::params.rigidTwoWayCoupling) {
        // Compute interactions and accumulate on rigid bodies
        applyRigidBoundaryCoupling(rigidBodies, true, dt);

        // Integrate rigid bodies using accumulated forces and gravity
        for (auto& body : rigidBodies) {
            body.applyForce(body.mass() * gravity);
            body.integrate(dt);
        }

        // After rigid moved, apply boundary response to fluid particles using updated samples
        applyRigidBoundaryCoupling(rigidBodies, false, dt);
    }
    else if (Sim::params.rigidOneWayCoupling) {
        // simple one-way: apply particle acceleration due to rigid samples (no forces to rigid)
        applyRigidBoundaryCoupling(rigidBodies, false, dt);
    }

    // When two-way coupling is disabled we still need to integrate rigid bodies (gravity, etc.).
    if (!rigidBodies.empty() && !Sim::params.rigidTwoWayCoupling) {
        for (auto& body : rigidBodies) {
            body.applyForce(body.mass() * gravity);
            body.integrate(dt);
        }
    }

    integrateFluid(dt);
    handleBoundaries();
    updateGrid();
}

void SPHSolver::handleBoundaries(){
    const int particleCount = static_cast<int>(particles.size());

    #ifdef _OPENMP
    #pragma omp parallel for schedule(static)
    for(int i = 0; i < particleCount; ++i){
        Particle &p = particles[i];
        for(int ax = 0; ax < 3; ++ax){
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
    #else
    for(int i = 0; i < particleCount; ++i){
        Particle &p = particles[i];
        for(int ax = 0; ax < 3; ++ax){
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
    #endif
}
