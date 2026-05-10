#include <glad/glad.h>
#include "core/Renderer.h"
#include "core/Window.h"
#include "ui/DebugUI.h"
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

    Sim::params.particleCount = (int)solver.getParticles().size();

    while (!window.shouldClose()) {
        window.pollEvents();

        if (Sim::params.resetRequested){
            solver = SPHSolver(Sim::params.h, particles);
            Sim::params.particleCount = (int)solver.getParticles().size();
            Sim::params.resetRequested = false;
        }
        if (!Sim::params.paused)
            solver.step(0.01f);

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