#include "Renderer.h"
#include <glad/glad.h>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <fstream>
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

void Renderer::init(){
    // Particle stuffs here
    
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
        
void Renderer::render(){
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
    
    drawBoundaryBox(m_view_proj);
}

void Renderer::shutdown(){
    glDeleteVertexArrays(1, &boxVAO);
    glDeleteBuffers(1, &boxVBO);
    glDeleteBuffers(1, &boxEBO);
    glDeleteProgram(boxShader);
}