#include "DebugUI.h"
#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>
#include <cmath>
#include "simulation/SimParams.h"

namespace {
void drawGravityIndicator() {
    const ImGuiIO& io = ImGui::GetIO();
    const ImVec2 displaySize = io.DisplaySize;
    const ImVec2 panelSize(120.f, 120.f);
    const ImVec2 panelPos(displaySize.x - panelSize.x - 16.f, 16.f);
    const ImVec2 center(panelPos.x + panelSize.x * 0.5f, panelPos.y + panelSize.y * 0.55f);

    const glm::vec3 g = Sim::params.gravityVector();
    glm::vec3 gn = glm::length(g) > 1e-6f ? glm::normalize(g) : glm::vec3(0.f, -1.f, 0.f);

    // Use X/Y on screen, and let Z influence brightness so the sign is still visible.
    ImVec2 dir(gn.x, -gn.y);
    const float dirLen = glm::clamp(glm::length(glm::vec2(gn.x, gn.y)), 0.15f, 1.f);
    if (glm::length(glm::vec2(dir.x, dir.y)) > 1e-6f) {
        const float invLen = 1.f / std::sqrt(dir.x * dir.x + dir.y * dir.y);
        dir.x *= invLen;
        dir.y *= invLen;
    } else {
        dir = ImVec2(0.f, 1.f);
    }

    const float arrowLen = 34.f * dirLen;
    const ImVec2 tip(center.x + dir.x * arrowLen, center.y + dir.y * arrowLen);
    const ImVec2 tail(center.x - dir.x * 18.f, center.y - dir.y * 18.f);
    const ImVec2 left(tip.x - dir.x * 8.f + dir.y * 5.f, tip.y - dir.y * 8.f - dir.x * 5.f);
    const ImVec2 right(tip.x - dir.x * 8.f - dir.y * 5.f, tip.y - dir.y * 8.f + dir.x * 5.f);

    const float brightness = glm::clamp(0.5f + 0.5f * gn.z, 0.2f, 1.f);
    const ImU32 bg = IM_COL32(20, 20, 24, 180);
    const ImU32 outline = IM_COL32(255, 255, 255, 45);
    const ImU32 arrow = IM_COL32((int)(255 * brightness), (int)(220 * brightness), (int)(120 * brightness), 255);
    const ImU32 text = IM_COL32(235, 235, 235, 220);

    ImDrawList* drawList = ImGui::GetForegroundDrawList();
    drawList->AddRectFilled(panelPos, ImVec2(panelPos.x + panelSize.x, panelPos.y + panelSize.y), bg, 10.f);
    drawList->AddRect(panelPos, ImVec2(panelPos.x + panelSize.x, panelPos.y + panelSize.y), outline, 10.f, 0, 1.5f);
    drawList->AddCircleFilled(center, 3.5f, text);
    drawList->AddLine(tail, tip, arrow, 3.0f);
    drawList->AddTriangleFilled(left, right, tip, arrow);
    drawList->AddText(ImVec2(panelPos.x + 12.f, panelPos.y + 10.f), text, "Gravity");
    drawList->AddText(ImVec2(panelPos.x + 12.f, panelPos.y + 28.f), text, "Yaw/Pitch");
}
}

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
    ImGui::Text("Particles: %d", Sim::params.fluidParticleCount);
    ImGui::Separator();

    if (ImGui::Button("Reset")) Sim::params.resetRequested = true;
    ImGui::SameLine();
    ImGui::Checkbox("Pause", &Sim::params.paused);

    ImGui::Spacing();

    // Sim Params
    if (ImGui::CollapsingHeader("Fluid", ImGuiTreeNodeFlags_DefaultOpen)) {
        int exactCount = Sim::params.fluidParticleCount;
        if (ImGui::InputInt("Exact Particle Count", &exactCount, 1, 100)) {
            if (exactCount < 1) exactCount = 1;
            if (exactCount > 30000) exactCount = 30000;
            Sim::params.fluidParticleCount = exactCount;
        }
        ImGui::SliderInt("Fluid Particle Count", &Sim::params.fluidParticleCount, 1, 30000);
        ImGui::SliderFloat("Particle Mass", &Sim::params.p_mass, 0.1f, 5.0f);
        ImGui::SliderFloat("Smoothing Radius (h)", &Sim::params.h,       0.01f,   0.5f);
        ImGui::SliderFloat("Rest Density",         &Sim::params.rho0,    100.f,  2000.f);
        ImGui::SliderFloat("Pressure Stiffness",   &Sim::params.k,       0.1f,    100.f);
        ImGui::SliderFloat("Viscosity",            &Sim::params.mu,      0.f,    10.f);
        ImGui::SliderFloat("Gravity Strength",     &Sim::params.gravity,  0.f,   20.f);
        ImGui::SliderFloat("Gravity Yaw",          &Sim::params.gravityYaw,   0.f, 360.f);
        ImGui::SliderFloat("Gravity Pitch",        &Sim::params.gravityPitch, -89.f,  89.f);
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

    drawGravityIndicator();
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