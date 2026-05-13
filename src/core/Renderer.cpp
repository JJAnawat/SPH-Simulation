#include "Renderer.h"
#include "simulation/Particle.h"
#include "simulation/RigidBody.h"
#include <glad/glad.h>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <fstream>
#include <limits>
#include <sstream>
#include <iostream>

unsigned int Renderer::compileShader(unsigned int type, const std::string& path) {
    std::ifstream file(path);
    if (!file.is_open()){
        std::cerr << "[RENDERER] Shader not found: " << path << '\n';
        return 0;
    }
    std::stringstream ss;
    ss << file.rdbuf();
    std::string src = ss.str();
    const char* c = src.c_str();

    unsigned int id = glCreateShader(type);
    glShaderSource(id, 1, &c, nullptr);
    glCompileShader(id);

    int ok;
    glGetShaderiv(id, GL_COMPILE_STATUS, &ok);
    if (!ok){
        char log[512];
        glGetShaderInfoLog(id, 512, nullptr, log);
        std::cerr << "[RENDERER] Compile Error: " << log << '\n';
        return 0;
    }
    return id;
}
        
unsigned int Renderer::linkProgram(unsigned int vert, unsigned int frag) {
    unsigned int prog = glCreateProgram();
    glAttachShader(prog, vert);
    glAttachShader(prog, frag);
    glLinkProgram(prog);

    int ok;
    glGetProgramiv(prog, GL_LINK_STATUS, &ok);
    if(!ok){
        char log[512];
        glGetProgramInfoLog(prog, 512, nullptr, log);
        std::cerr << "[RENDERER] Link error: " << log << '\n';
    }
    glDeleteShader(vert);
    glDeleteShader(frag);
    return prog;
}
        
void Renderer::drawBoundaryBox(const glm::mat4& m_view_proj) { 
    glUseProgram(boxShader);
    glUniformMatrix4fv(glGetUniformLocation(boxShader, "u_mat_view_proj"), 1, GL_FALSE, glm::value_ptr(m_view_proj));
    glBindVertexArray(boxVAO);
    glDrawElements(GL_LINES, 24, GL_UNSIGNED_INT, 0);
    glBindVertexArray(0);
}

void Renderer::drawParticles(const std::vector<Particle>& particles, const glm::mat4& m_view_proj){
    if(particles.empty())
        return;
    
    glBindVertexArray(particleVAO);
    glBindBuffer(GL_ARRAY_BUFFER, particleVBO);
    glBufferData(GL_ARRAY_BUFFER, (GLsizeiptr)(particles.size() * sizeof(Particle)), particles.data(), GL_DYNAMIC_DRAW);

    glUseProgram(particleShader);
    glUniformMatrix4fv(glGetUniformLocation(particleShader, "u_mat_view_proj"), 1, GL_FALSE, glm::value_ptr(m_view_proj));
    glUniform1f(glGetUniformLocation(particleShader, "u_point_size"), Sim::params.pointSize);
    glUniform3fv(glGetUniformLocation(particleShader, "u_color"), 1, Sim::params.particleColor);
    glUniform1i(glGetUniformLocation(particleShader, "u_velocity_coloring"), Sim::params.velocityColoring);
    
    // This max speed finding is still weird because if the next frame velocity all shift up by 10m/s it would still render the same color
    // TODO: fix this thing
    float maxSpeed = 0.f;
    for(auto& p: particles)
        maxSpeed = std::max(maxSpeed, glm::length(p.velocity));
    glUniform1f(glGetUniformLocation(particleShader, "u_max_speed"), maxSpeed > 0.f ? maxSpeed : 1.f);

    glDrawArrays(GL_POINTS, 0, (GLsizei) particles.size());
    glBindVertexArray(0);
}

void Renderer::init(){
    // Particles
    particleShader = linkProgram(
        compileShader(GL_VERTEX_SHADER, "shaders/particle.vert"),
        compileShader(GL_FRAGMENT_SHADER, "shaders/particle.frag")
    );

    // Particle VAO - [pos(12)][vel(12)][density(4)][pressure(4)]
    glGenVertexArrays(1, &particleVAO);
    glGenBuffers(1, &particleVBO);
    glBindVertexArray(particleVAO);
    glBindBuffer(GL_ARRAY_BUFFER, particleVBO);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Particle), (void *)offsetof(Particle, position));
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Particle), (void *)offsetof(Particle, velocity));
    glEnableVertexAttribArray(1);
    glBindVertexArray(0);
    
    // Rigid body samples VAO
    glGenVertexArrays(1, &rigidSampleVAO);
    glGenBuffers(1, &rigidSampleVBO);
    glBindVertexArray(rigidSampleVAO);
    glBindBuffer(GL_ARRAY_BUFFER, rigidSampleVBO);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(glm::vec3), (void*)0);
    glEnableVertexAttribArray(0);
    glBindVertexArray(0);
    
    // Bounding box
    boxShader = linkProgram(
        compileShader(GL_VERTEX_SHADER, "shaders/box.vert"),
        compileShader(GL_FRAGMENT_SHADER, "shaders/box.frag")
    );
    
    float x0 = Sim::params.boxMin[0], y0 = Sim::params.boxMin[1], z0 = Sim::params.boxMin[2];
    float x1 = Sim::params.boxMax[0], y1 = Sim::params.boxMax[1], z1 = Sim::params.boxMax[2];

    float verts[] = {
        x0,y0,z0,  x1,y0,z0,  x1,y0,z1,  x0,y0,z1,  // bottom
        x0,y1,z0,  x1,y1,z0,  x1,y1,z1,  x0,y1,z1,  // top
    };

    unsigned int idx[] = {
        0,1, 1,2, 2,3, 3,0,   // bottom face
        4,5, 5,6, 6,7, 7,4,   // top face
        0,4, 1,5, 2,6, 3,7    // verticals
    };

    glGenVertexArrays(1, &boxVAO);
    glGenBuffers(1, &boxVBO);
    glGenBuffers(1, &boxEBO);

    glBindVertexArray(boxVAO);
    glBindBuffer(GL_ARRAY_BUFFER, boxVBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(verts), verts, GL_STATIC_DRAW);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, boxEBO);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(idx), idx, GL_STATIC_DRAW);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);
    glBindVertexArray(0);

    glEnable(GL_PROGRAM_POINT_SIZE);   
    glEnable(GL_DEPTH_TEST);
}
        
void Renderer::render(const std::vector<Particle>& particles){
    glClearColor(0.1f, 0.1f, 0.1f, 1.f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // Simple orbit camera
    float yaw   = glm::radians(Sim::camera.yaw);
    float pitch = glm::radians(Sim::camera.pitch);
    float dist = Sim::camera.distance;
    glm::vec3 eye = glm::vec3(
        dist * cos(pitch) * sin(yaw),
        dist * sin(pitch),
        dist * cos(pitch) * cos(yaw)
    );
    glm::mat4 view = glm::lookAt(eye, glm::vec3(0.f), glm::vec3(0,1,0));
    glm::mat4 proj = glm::perspective(glm::radians(45.f), aspectRatio, 0.1f, 100.f);
    glm::mat4 m_view_proj  = proj * view;
    
    drawParticles(particles, m_view_proj);
    drawBoundaryBox(m_view_proj);
}

void Renderer::renderWithRigidBodies(const std::vector<Particle>& particles, const std::vector<RigidBody>& bodies) {
    glClearColor(0.1f, 0.1f, 0.1f, 1.f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    float yaw   = glm::radians(Sim::camera.yaw);
    float pitch = glm::radians(Sim::camera.pitch);
    float dist = Sim::camera.distance;
    glm::vec3 eye = glm::vec3(
        dist * cos(pitch) * sin(yaw),
        dist * sin(pitch),
        dist * cos(pitch) * cos(yaw)
    );
    glm::mat4 view = glm::lookAt(eye, glm::vec3(0.f), glm::vec3(0,1,0));
    glm::mat4 proj = glm::perspective(glm::radians(45.f), aspectRatio, 0.1f, 100.f);
    glm::mat4 m_view_proj  = proj * view;

    drawParticles(particles, m_view_proj);
    drawBoundaryBox(m_view_proj);

    float minDensity = std::numeric_limits<float>::max();
    float maxDensity = std::numeric_limits<float>::lowest();
    for (const auto& body : bodies) {
        minDensity = std::min(minDensity, body.density());
        maxDensity = std::max(maxDensity, body.density());
    }
    const float densityRange = std::max(1e-6f, maxDensity - minDensity);

    for (size_t i = 0; i < bodies.size(); ++i) {
        const float t = (bodies[i].density() - minDensity) / densityRange;
        const float color[3] = {
            0.25f + 0.75f * t,
            0.90f - 0.45f * t,
            0.35f + 0.55f * (1.f - t)
        };
        drawRigidBody(bodies[i], m_view_proj, color, t);
    }
}

void Renderer::drawRigidBody(const RigidBody& body, const glm::mat4& m_view_proj, const float color[3], float densityT) {
    const auto& samples = body.worldSamples();
    if (samples.empty()) return;

    glUseProgram(particleShader);
    glUniformMatrix4fv(glGetUniformLocation(particleShader, "u_mat_view_proj"), 1, GL_FALSE, glm::value_ptr(m_view_proj));
    glUniform1f(glGetUniformLocation(particleShader, "u_point_size"), Sim::params.pointSize * (1.15f + 0.45f * densityT));

    glUniform3fv(glGetUniformLocation(particleShader, "u_color"), 1, color);
    glUniform1i(glGetUniformLocation(particleShader, "u_velocity_coloring"), 0);
    glUniform1f(glGetUniformLocation(particleShader, "u_max_speed"), 1.f);

    glBindVertexArray(rigidSampleVAO);
    glBindBuffer(GL_ARRAY_BUFFER, rigidSampleVBO);
    glBufferData(GL_ARRAY_BUFFER, (GLsizeiptr)(samples.size() * sizeof(glm::vec3)), samples.data(), GL_DYNAMIC_DRAW);
    
    glDrawArrays(GL_POINTS, 0, (GLsizei)samples.size());
    glBindVertexArray(0);
}

void Renderer::shutdown(){
    glDeleteVertexArrays(1, &particleVAO);
    glDeleteBuffers(1, &particleVBO);
    glDeleteProgram(particleShader);
    glDeleteVertexArrays(1, &boxVAO);
    glDeleteBuffers(1, &boxVBO);
    glDeleteBuffers(1, &boxEBO);
    glDeleteProgram(boxShader);
    glDeleteVertexArrays(1, &rigidSampleVAO);
    glDeleteBuffers(1, &rigidSampleVBO);
}