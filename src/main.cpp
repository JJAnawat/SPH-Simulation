#include <glad/glad.h>
#include "core/Renderer.h"
#include "core/Window.h"
#include "ui/DebugUI.h"
#include "simulation/RigidBody.h"
#include "simulation/SPHSolver.h"
#include "simulation/ParticleUtils.h"

#include <algorithm>
#include <cfloat>

static void resolveRigidBodyBoxCollision(RigidBody& body) {
    const auto& samples = body.worldSamples();
    if (samples.empty()) {
        return;
    }

    glm::vec3 minPoint(FLT_MAX);
    glm::vec3 maxPoint(-FLT_MAX);
    for (const auto& sample : samples) {
        minPoint = glm::min(minPoint, sample);
        maxPoint = glm::max(maxPoint, sample);
    }

    glm::vec3 correction(0.f);
    glm::vec3 velocity = body.linearVelocity();
    bool collided = false;

    for (int axis = 0; axis < 3; ++axis) {
        if (minPoint[axis] < Sim::params.boxMin[axis]) {
            correction[axis] += Sim::params.boxMin[axis] - minPoint[axis];
            if (velocity[axis] < 0.f) {
                velocity[axis] *= -Sim::params.rigidWallBounce;
            }
            collided = true;
        }
        if (maxPoint[axis] > Sim::params.boxMax[axis]) {
            correction[axis] -= maxPoint[axis] - Sim::params.boxMax[axis];
            if (velocity[axis] > 0.f) {
                velocity[axis] *= -Sim::params.rigidWallBounce;
            }
            collided = true;
        }
    }

    if (collided) {
        body.setPosition(body.position() + correction);
        body.setLinearVelocity(velocity);
        body.setAngularVelocity(body.angularVelocity() * Sim::params.rigidAngularWallDamping);
    }
}

int main() {
    Window window(1280, 720, "SPH Fluid Simulation");
    
    DebugUI ui;
    ui.init(window.getHandle());
    
    Renderer renderer;
    renderer.init();

    auto particles = makeSphere({0.f, 1.5f, 0.f}, 0.3f, 1000);
    SPHSolver solver(Sim::params.h, particles);
    auto rigidBody = RigidBody::createBox({0.f, 0.75f, 0.f}, {0.35f, 0.18f, 0.25f}, 2.5f, 5);
    std::vector<RigidBody> rigidBodies;
    rigidBodies.push_back(rigidBody);

    Sim::params.particleCount = (int)solver.getParticles().size();

    while (!window.shouldClose()) {
        window.pollEvents();

        if (Sim::params.resetRequested){
            solver = SPHSolver(Sim::params.h, particles);
            rigidBodies[0] = RigidBody::createBox({0.f, 0.75f, 0.f}, {0.35f, 0.18f, 0.25f}, 2.5f, 5);
            Sim::params.particleCount = (int)solver.getParticles().size();
            Sim::params.resetRequested = false;
        }
        if (!Sim::params.paused) {
            solver.step(0.01f, rigidBodies);
            // Rigid bodies are integrated inside solver when two-way coupling is enabled.
            resolveRigidBodyBoxCollision(rigidBodies[0]);
        }

        if (Sim::params.showRigidBodies)
            renderer.renderWithRigidBodies(solver.getParticles(), rigidBodies);
        else
            renderer.render(solver.getParticles());

        // ImGui
        ui.beginFrame();
        ui.render();
        ui.endFrame();

        window.swapBuffers();
    }

    renderer.shutdown();
    ui.shutdown();
    return 0;
}