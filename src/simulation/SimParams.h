#pragma once

struct CameraParams {
    float distance = 10.f;
    float yaw = 0.f;
    float pitch = 18.f;
};

struct SimParams {
    // SPU Params
    float h = 0.5f; // smoothing radius
    float rho0 = 1000.f; // rest density
    float k = 200.f; // pressure stiffness
    float mu = 0.1f; // viscosity

    // World
    float gravity = -9.8f;
    float damp = 0.7f;

    // Control flags
    bool paused = false;
    bool resetRequested = false;
    bool showRigidBodies = true;
    bool rigidOneWayCoupling = true;
    bool rigidTwoWayCoupling = false;
    int particleCount = 0;

    // Rigid-fluid coupling (one-way for now)
    float rigidBoundaryStiffness = 80.f;
    float rigidBoundaryDamping = 6.f;

    // Rigid-box collision
    float rigidWallBounce = 0.5f;
    float rigidAngularWallDamping = 0.8f;

    // Box
    float boxMin[3] = {-4.f, -2.f, -2.f};
    float boxMax[3] = { 4.f,  2.f,  2.f};
    
    // Visualize
    float pointSize        = 8.f;
    float particleColor[3] = {0.2f, 0.6f, 1.f};
    bool velocityColoring = true;
};

namespace Sim {
    inline SimParams params;
    inline CameraParams camera;
}