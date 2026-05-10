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

    while (!window.shouldClose()) {
        window.pollEvents();

        // if (!params.paused)
        //     solver.step(params);

        renderer.render();

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