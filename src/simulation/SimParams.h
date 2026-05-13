#pragma once

struct CameraParams {
    float distance = 2.f;
    float yaw = 0.f;
    float pitch = 18.f;
};

struct SimParams {
    // SPU Params
    float h = 0.1f; // smoothing radius
    float rho0 = 1000.f; // rest density
    float k = 2.f; // pressure stiffness
    float mu = 2.f; // viscosity
    float p_mass = 1.f; // particle mass

    // World
    float gravity = -9.8f;
    float damp = 0.3f;

    // Control flags
    bool paused = false;
    bool resetRequested = false;
    bool showRigidBodies = false;
    bool rigidOneWayCoupling = false;
    bool rigidTwoWayCoupling = false;
    int fluidParticleCount = 1000;

    // Rigid-fluid coupling (one-way for now)
    float rigidBoundaryStiffness = 18.f;
    float rigidBoundaryDamping = 12.f;

    // Rigid-box collision
    float rigidWallBounce = 0.15f;
    float rigidAngularWallDamping = 0.55f;

    // Box
    float boxMin[3] = {-0.5f, -0.5f, -0.5f};
    float boxMax[3] = { 0.5f,  0.5f,  0.5f};
    
    // Visualize
    float pointSize        = 8.f;
    float particleColor[3] = {0.2f, 0.6f, 1.f};
    bool velocityColoring = true;
};

namespace Sim {
    inline SimParams params;
    inline CameraParams camera;
}