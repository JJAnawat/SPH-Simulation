#include <glad/glad.h>
#include "core/Renderer.h"
#include "core/Window.h"
#include "ui/DebugUI.h"
#include "simulation/RigidBody.h"
#include "simulation/SPHSolver.h"
#include "simulation/ParticleUtils.h"

int main() {
    Window window(1280, 720, "SPH Fluid Simulation");
    
    DebugUI ui;
    ui.init(window.getHandle());
    
    Renderer renderer;
    renderer.init();

    auto particles = makeSphere({0.f, 1.5f, 0.f}, 0.3f, 500);
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
            solver.step(0.01f);
            
            // Apply gravity to rigid body (for now, independent of coupling)
            rigidBodies[0].applyForce({0.f, rigidBodies[0].mass() * -9.8f, 0.f});
            rigidBodies[0].integrate(0.01f);
            
            // Handle boundaries for rigid body
            const auto& pos = rigidBodies[0].position();
            const float radius = 0.35f;
            if (pos.x - radius < Sim::params.boxMin[0] || pos.x + radius > Sim::params.boxMax[0] ||
                pos.y - radius < Sim::params.boxMin[1] || pos.y + radius > Sim::params.boxMax[1] ||
                pos.z - radius < Sim::params.boxMin[2] || pos.z + radius > Sim::params.boxMax[2]) {
                rigidBodies[0].setLinearVelocity(rigidBodies[0].linearVelocity() * glm::vec3(0.7f));
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