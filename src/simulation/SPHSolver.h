#ifndef SPHSOLVER_H
#define SPHSOLVER_H

#include "simulation/Particle.h"
#include "simulation/SpatialHash.h"
#include <vector>

struct SPHSolver {
    public:
        SPHSolver(const float h, std::vector<Particle> parts) : grid(h), particles(parts) {}
        void step(float dt);

        const std::vector<Particle>& getParticles() const { return particles; }

    private:
        void computeDensityPressure();
        void computeForces();
        void handleBoundaries();
        void updateGrid();

        std::vector<Particle> particles;
        SpatialHash grid;
};

#endif