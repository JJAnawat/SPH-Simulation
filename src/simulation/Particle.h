#pragma once
#include <glm/glm.hpp>

struct Particle{
    glm::vec3 position, velocity, acceleration;
    float density, pressure, mass;
};  