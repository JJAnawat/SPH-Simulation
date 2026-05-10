#pragma once
#include <glm/glm.hpp>

struct Particle{
    glm::vec3 position, velocity;
    float density, pressure;
};  