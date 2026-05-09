#pragma once

struct SimParams {
    // SPU Params
    float h = 0.5f; // smoothing radius
    float rho0 = 1000.f; // rest density
    float k = 200.f; // pressure stiffness
    float mu = 0.1f; // viscosity

    // World
    float gravity = -9.8f;

    // Control flags
    bool paused = false;
    bool resetRequested = false;
    int particleCount = 0;
};