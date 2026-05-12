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

void DebugUI::render() {
    ImGui::Begin("SPH Debug");
    
    // Info
    ImGui::Text("FPS: %.1f", ImGui::GetIO().Framerate);
    ImGui::Text("Particles: %d", Sim::params.particleCount);
    ImGui::Separator();

    if (ImGui::Button("Reset")) Sim::params.resetRequested = true;
    ImGui::SameLine();
    ImGui::Checkbox("Pause", &Sim::params.paused);

    ImGui::Spacing();

    // Sim Params
    if (ImGui::CollapsingHeader("Fluid", ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::SliderFloat("Particle Mass", &Sim::params.p_mass, 0.1f, 5.0f);
        ImGui::SliderFloat("Smoothing Radius (h)", &Sim::params.h,       0.01f,   0.5f);
        ImGui::SliderFloat("Rest Density",         &Sim::params.rho0,    100.f,  2000.f);
        ImGui::SliderFloat("Pressure Stiffness",   &Sim::params.k,       0.1f,    100.f);
        ImGui::SliderFloat("Viscosity",            &Sim::params.mu,      0.f,    10.f);
        ImGui::SliderFloat("Gravity",              &Sim::params.gravity, -20.f,  0.f);
    }

    if (ImGui::CollapsingHeader("Rigid Body Coupling")) {
        ImGui::Checkbox("One-Way Coupling", &Sim::params.rigidOneWayCoupling);
        ImGui::Checkbox("Two-Way Coupling", &Sim::params.rigidTwoWayCoupling);
        ImGui::Checkbox("Show Rigid Bodies", &Sim::params.showRigidBodies);

        ImGui::Spacing();
        ImGui::SliderFloat("Boundary Stiffness",  &Sim::params.rigidBoundaryStiffness,   0.f, 300.f);
        ImGui::SliderFloat("Boundary Damping",    &Sim::params.rigidBoundaryDamping,     0.f,  20.f);
        ImGui::SliderFloat("Wall Bounce",         &Sim::params.rigidWallBounce,          0.f,   1.f);
        ImGui::SliderFloat("Angular Wall Damp",   &Sim::params.rigidAngularWallDamping,  0.f,   1.f);
    }

    if (ImGui::CollapsingHeader("Rendering")) {
        ImGui::Checkbox("Velocity Coloring", &Sim::params.velocityColoring);
        ImGui::SliderFloat("Cam Yaw",      &Sim::camera.yaw,      0.f,   360.f);
        ImGui::SliderFloat("Cam Pitch",    &Sim::camera.pitch,    -89.f,  89.f);
        ImGui::SliderFloat("Cam Distance", &Sim::camera.distance,  1.f,   30.f);
    }

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