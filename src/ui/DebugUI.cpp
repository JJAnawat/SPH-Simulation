#include "DebugUI.h"
#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>
#include "simulation/SimParams.h"

void DebugUI::init(GLFWwindow* window) {
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGui::StyleColorsDark();
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init("#version 330");
}

void DebugUI::beginFrame() {
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();
}

void DebugUI::render(SimParams& params) {
    ImGui::Begin("SPH Debug");
    
    // Info
    ImGui::Text("FPS: %.1f", ImGui::GetIO().Framerate);
    ImGui::Text("Particles: %d", params.particleCount);
    ImGui::Separator();

    // Sim Params
    ImGui::SeparatorText("Fluid");
    ImGui::SliderFloat("Smoothing Radius (h)", &params.h,     0.1f, 2.0f);
    ImGui::SliderFloat("Rest Density",         &params.rho0,  500.f, 2000.f);
    ImGui::SliderFloat("Pressure Stiffness",   &params.k,     1.f,  500.f);
    ImGui::SliderFloat("Viscosity",            &params.mu,    0.f,  1.f);

    ImGui::SeparatorText("World");
    ImGui::SliderFloat("Gravity",              &params.gravity, -20.f, 0.f);

    // Controls
    ImGui::Separator();
    if (ImGui::Button("Reset Simulation"))
        params.resetRequested = true;

    ImGui::SameLine(); // put next widget on same line
    ImGui::Checkbox("Pause", &params.paused);

    ImGui::End();
}

void DebugUI::endFrame() {
    ImGui::Render();
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
}

void DebugUI::shutdown() {
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
}