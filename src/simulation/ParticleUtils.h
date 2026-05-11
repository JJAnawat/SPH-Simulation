#pragma once

#include "Particle.h"
#include <glm/glm.hpp>
#include <vector>
#include <random>
#include <algorithm>

const double PI = std::acos(-1.0);

inline std::vector<Particle> makeBlock(
    const glm::vec3& origin,
    int countX, int countY, int countZ,
    float spacing
){
    std::vector<Particle> particles;
    particles.reserve(countX * countY * countZ);

    int   N      = countX * countY * countZ;
    float volume = (countX * spacing) * (countY * spacing) * (countZ * spacing);
    float mass   = Sim::params.rho0 * (volume / N);

    for(int x=0; x<countX; x++){
        for(int y=0; y<countY; y++){
            for(int z=0; z<countZ; z++){
                Particle p;
                p.mass = mass;
                p.position = origin + glm::vec3(x,y,z) * spacing;
                p.velocity = glm::vec3(0.f);
                p.acceleration = glm::vec3(0.f);
                p.density = 0.f;
                p.pressure = 0.f;
                particles.push_back(p);
            }
        }
    }
    return particles;
}

inline std::vector<Particle> makeSphere(
    const glm::vec3& center,
    float radius, int count
){
    std::vector<Particle> particles;
    particles.reserve(count);
    
    std::mt19937 rng(67);
    std::uniform_real_distribution<float> dist(0.f, 1.f);

    float volume = (4.0f / 3.0f) * PI * radius * radius * radius;
    float mass   = Sim::params.rho0 * (volume / count);
    
    while((int)particles.size() < count){
        glm::vec3 offset = {
            dist(rng) * 2.f - 1.f,
            dist(rng) * 2.f - 1.f,
            dist(rng) * 2.f - 1.f
        };

        if(glm::length(offset) > 1.f) continue;
        
        Particle p;
        p.mass = mass;
        p.position = center + offset * radius;
        p.velocity = glm::vec3(0.f);
        p.acceleration = glm::vec3(0.f);
        p.density = 0.f;
        p.pressure = 0.f;
        particles.push_back(p);
    }
    return particles;
}

inline std::vector<glm::vec3> extractPos(std::vector<Particle>& parts){
    std::vector<glm::vec3> pos_vec;
    std::transform(parts.begin(), parts.end(), std::back_inserter(pos_vec),
                    [](const Particle& p) {return p.position; });
    return pos_vec;
}