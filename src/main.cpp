#include <glad/glad.h>
#include "core/Renderer.h"
#include "core/Window.h"
#include "ui/DebugUI.h"
#include "simulation/SPHSolver.h"
#include "simulation/RigidBodyUtils.h"
#include "simulation/ParticleUtils.h"

int main() {
    Window window(1280, 720, "SPH Fluid Simulation");
    
    DebugUI ui;
    ui.init(window.getHandle());
    
    Renderer renderer;
    renderer.init();

    auto particles = createFluidParticles(Sim::params.fluidParticleCount, "block");
    SPHSolver solver(Sim::params.h, particles);
    std::vector<RigidBody> rigidBodies = createRigidBodies();

    Sim::params.showRigidBodies = true;
    Sim::params.rigidTwoWayCoupling = true;

    while (!window.shouldClose()) {
        window.pollEvents();

        if (Sim::params.resetRequested){
            particles = createFluidParticles(Sim::params.fluidParticleCount, "block");
            solver = SPHSolver(Sim::params.h, particles);
            rigidBodies = createRigidBodies();
            Sim::params.resetRequested = false;
        }
        if (!Sim::params.paused) {
            solver.step(0.01f, rigidBodies);
            // Rigid bodies are integrated inside solver when two-way coupling is enabled.
            for (auto& body : rigidBodies) {
                resolveRigidBodyBoxCollision(body);
            }
            resolveRigidBodyPairCollisions(rigidBodies);
            for (auto& body : rigidBodies) {
                resolveRigidBodyBoxCollision(body);
            }
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