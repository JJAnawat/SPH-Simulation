// From this dude https://matthias-research.github.io/pages/publications/sca03.pdf
#pragma once

#include "SimParams.h"
#include <glm/glm.hpp>
#include <vector>
#include <algorithm>

const double PI = std::acos(-1.0);

float w_poly6(glm::vec3 r){
    return (315.f / (64.f * PI * pow(Sim::params.h, 9))) * pow(Sim::params.h * Sim::params.h - glm::length(r) * glm::length(r), 3);
}

float grad_w_spiky(glm::vec3 r){
    return (-45.f / (PI * pow(Sim::params.h, 6))) * pow(Sim::params.h - glm::length(r), 2);
}

float laplacian_w_viscosity(glm::vec3 r){
    return (45.f / (PI * pow(Sim::params.h, 6))) * (Sim::params.h - glm::length(r));
}