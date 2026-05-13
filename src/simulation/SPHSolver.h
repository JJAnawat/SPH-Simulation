#ifndef SPHSOLVER_H
#define SPHSOLVER_H

#include "simulation/Particle.h"
#include "simulation/RigidBody.h"
#include "simulation/SpatialHash.h"
#include <vector>

struct SPHSolver {
    public:
        SPHSolver(const float h, std::vector<Particle> parts) : grid(h), particles(parts) {}
        void step(float dt);
        void step(float dt, std::vector<RigidBody>& rigidBodies);

        const std::vector<Particle>& getParticles() const { return particles; }

    private:
        void computeDensityPressure();
        void computeForces();
        void applyRigidBoundaryCoupling(std::vector<RigidBody>& rigidBodies, bool accumulateToRigid, float dt);
        void integrateFluid(float dt);
        void handleBoundaries();
        void updateGrid();

        std::vector<Particle> particles;
        SpatialHash grid;
};

#endif