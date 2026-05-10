#ifndef SPHSOLVER_H
#define SPHSOLVER_H

#include "simulation/Particle.h"
#include "simulation/SpatialHash.h"
#include <vector>

struct SPHSolver {
    public:
        SPHSolver(const float h, std::vector<Particle> parts);
        void step(float dt);

        const std::vector<Particle>& getParticles() const { return particles; }

    private:
        void computeDensityPressure();
        void computeForces();
        void handleBoundaries();

        std::vector<Particle> particles;
        SpatialHash grid;
};

#endif