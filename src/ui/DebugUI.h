#pragma once
#include <GLFW/glfw3.h>

class DebugUI {
public:
    void init(GLFWwindow* window);
    void beginFrame();
    void render();
    void endFrame();
    void shutdown();
};