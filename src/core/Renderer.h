#ifndef RENDERER_H
#define RENDERER_H

#include <string>
#include <vector>
#include <glm/glm.hpp>
#include "simulation/SimParams.h"
#include "simulation/Particle.h"

class Renderer {
    private:
        unsigned int boxShader = 0;
        unsigned int particleShader = 0;
        
        unsigned int boxVAO = 0, boxVBO = 0, boxEBO = 0;
        unsigned int particleVAO = 0, particleVBO = 0;

        unsigned int compileShader(unsigned int type, const std::string& path);
        unsigned int linkProgram(unsigned int vert, unsigned int frag);
        void drawParticles(const std::vector<Particle>& particles, const glm::mat4& m_view_proj);
        void drawBoundaryBox(const glm::mat4& m_view_proj);

    public:
        void init();
        void render(const std::vector<Particle>& particles);
        void shutdown();

        float aspectRatio = 1280.f / 720.f;
};

#endif