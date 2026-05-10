#include <glad/glad.h>
#include "core/Window.h"
#include "ui/DebugUI.h"

int main() {
    Window window(1280, 720, "SPH Fluid Simulation");
    DebugUI ui;
    ui.init(window.getHandle());

    while (!window.shouldClose()) {
        window.pollEvents();

        // if (!params.paused)
        //     solver.step(params);

        // Clear
        glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);

        // ImGui
        ui.beginFrame();
        ui.render();
        ui.endFrame();

        window.swapBuffers();
    }

    ui.shutdown();
    return 0;
}