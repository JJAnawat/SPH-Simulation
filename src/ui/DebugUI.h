#pragma once
#include <GLFW/glfw3.h>
#include "simulation/SimParams.h"

class DebugUI {
public:
    void init(GLFWwindow* window);
    void beginFrame();
    void render(SimParams& params);
    void endFrame();
    void shutdown();
};