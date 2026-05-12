// From this dude https://matthias-research.github.io/pages/publications/sca03.pdf
#pragma once

#include "SimParams.h"
#include <glm/glm.hpp>
#include <glm/gtc/constants.hpp>

inline float w_poly6(float r2) {
    const float h = Sim::params.h;
    const float h2 = h * h;
    const float diff = h2 - r2;
    if (diff <= 0.f) {
        return 0.f;
    }

    const float h9 = h2 * h2 * h2 * h;
    const float coeff = 315.f / (64.f * glm::pi<float>() * h9);
    return coeff * diff * diff * diff;
}

inline float grad_w_spiky(float r_len) {
    const float h = Sim::params.h;
    const float diff = h - r_len;
    if (diff <= 0.f) {
        return 0.f;
    }

    const float h6 = h * h * h * h * h * h;
    const float coeff = -45.f / (glm::pi<float>() * h6);
    return coeff * diff * diff;
}

inline float laplacian_w_viscosity(float r_len) {
    const float h = Sim::params.h;
    const float diff = h - r_len;
    if (diff <= 0.f) {
        return 0.f;
    }

    const float h6 = h * h * h * h * h * h;
    const float coeff = 45.f / (glm::pi<float>() * h6);
    return coeff * diff;
}