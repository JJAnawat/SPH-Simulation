#pragma once

#include "Particle.h"
#include "SimParams.h"
#include <glm/glm.hpp>
#include <glm/gtc/constants.hpp>
#include <vector>
#include <random>

inline std::vector<Particle> makeBlock(
    const glm::vec3& origin,
    int countX, int countY, int countZ,
    float spacing
){
    std::vector<Particle> particles;
    particles.reserve(countX * countY * countZ);

    for(int x=0; x<countX; x++){
        for(int y=0; y<countY; y++){
            for(int z=0; z<countZ; z++){
                Particle p;
                p.mass = Sim::params.p_mass;
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

    while((int)particles.size() < count){
        glm::vec3 offset = {
            dist(rng) * 2.f - 1.f,
            dist(rng) * 2.f - 1.f,
            dist(rng) * 2.f - 1.f
        };

        if(glm::length(offset) > 1.f) continue;
        
        Particle p;
        p.mass = Sim::params.p_mass;
        p.position = center + offset * radius;
        p.velocity = glm::vec3(0.f);
        p.acceleration = glm::vec3(0.f);
        p.density = 0.f;
        p.pressure = 0.f;
        particles.push_back(p);
    }
    return particles;
}

inline std::vector<Particle> makeRandomParticles(
    float min_p, float max_p, int count
){
    std::vector<Particle> particles;
    particles.reserve(count);

    std::mt19937 rng(67);
    std::uniform_real_distribution<float> dist(min_p, max_p);

    while((int)particles.size() < count){
        float x = dist(rng);
        float y = dist(rng);
        float z = dist(rng);
        
        Particle p;
        p.mass = Sim::params.p_mass;
        p.position = glm::vec3(x, y, z);
        p.velocity = glm::vec3(0.f);
        p.acceleration = glm::vec3(0.f);
        p.density = 0.f;
        p.pressure = 0.f;
        particles.push_back(p);
    }
    return particles;
}

inline std::vector<Particle> createFluidParticles(int count, std::string shape = "block") {
    count = std::max(count, 1);

    if(shape == "block"){
        const int countPerAxis = std::max(1, static_cast<int>(std::ceil(std::cbrt(static_cast<double>(count)))));
        const int layerSize = countPerAxis * countPerAxis;
        const int layers = std::max(1, (count + layerSize - 1) / layerSize);
        const float spacing = 0.03f;
        const glm::vec3 origin = {
            -0.5f * spacing * static_cast<float>(countPerAxis - 1),
            -0.18f,
            -0.5f * spacing * static_cast<float>(countPerAxis - 1)
        };
    
        return makeBlock(origin, countPerAxis, countPerAxis, layers, spacing);
    }
    else if(shape == "sphere"){
        const float radius = 0.3f;
        const glm::vec3 center(0.f, 0.f, 0.f);
        return makeSphere(center, radius, count);
    }
    else{
        return makeRandomParticles(-0.5f, 0.5f, count);
    }
}

inline std::vector<glm::vec3> extractPos(const std::vector<Particle>& parts){
    std::vector<glm::vec3> pos_vec;
    pos_vec.reserve(parts.size());
    std::transform(parts.begin(), parts.end(), std::back_inserter(pos_vec),
                    [](const Particle& p) {return p.position; });
    return pos_vec;
}
