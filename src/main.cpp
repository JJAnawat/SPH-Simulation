#include <glad/glad.h>
#include "core/Renderer.h"
#include "core/Window.h"
#include "ui/DebugUI.h"

int main() {
    Window window(1280, 720, "SPH Fluid Simulation");
    
    DebugUI ui;
    ui.init(window.getHandle());
    
    Renderer renderer;
    renderer.init();

    std::vector<Particle> particles(2);
    particles[0].position = glm::vec3(0.f, 1.f, 0.f);
    particles[1].position = glm::vec3(0.f, 1.f, 1.f);
    particles[1].velocity = glm::vec3(1.f, 1.f, 1.f);

    Sim::params.particleCount = (int)particles.size();

    while (!window.shouldClose()) {
        window.pollEvents();

        // if (!params.paused)
        //     solver.step(params);

        renderer.render(particles);

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