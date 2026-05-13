#pragma once 

#include "RigidBody.h"
#include "SimParams.h"
#include <vector>

inline std::vector<RigidBody> createRigidBodies() {
    std::vector<RigidBody> bodies;
    bodies.emplace_back(RigidBody::createBox({-0.18f, 0.80f, 0.00f}, {0.11f, 0.065f, 0.08f}, 0.38f, 7));
    bodies.emplace_back(RigidBody::createBox({ 0.14f, 0.93f,-0.05f}, {0.085f, 0.050f, 0.065f}, 125.54f, 8));
    bodies.emplace_back(RigidBody::createSphere({ 0.02f, 0.72f, 0.10f}, 0.060f, 0.28f, 8, 16));
    return bodies;
}

inline bool resolveRigidBodyPairCollision(RigidBody& bodyA, RigidBody& bodyB) {
    const auto& samplesA = bodyA.worldSamples();
    const auto& samplesB = bodyB.worldSamples();
    if (samplesA.empty() || samplesB.empty()) {
        return false;
    }

    glm::vec3 minA(FLT_MAX), maxA(-FLT_MAX);
    glm::vec3 minB(FLT_MAX), maxB(-FLT_MAX);
    for (const auto& s : samplesA) {
        minA = glm::min(minA, s);
        maxA = glm::max(maxA, s);
    }
    for (const auto& s : samplesB) {
        minB = glm::min(minB, s);
        maxB = glm::max(maxB, s);
    }

    const glm::vec3 overlap = glm::min(maxA, maxB) - glm::max(minA, minB);
    if (overlap.x <= 0.f || overlap.y <= 0.f || overlap.z <= 0.f) {
        return false;
    }

    int axis = 0;
    float minOverlap = overlap.x;
    if (overlap.y < minOverlap) {
        axis = 1;
        minOverlap = overlap.y;
    }
    if (overlap.z < minOverlap) {
        axis = 2;
        minOverlap = overlap.z;
    }

    glm::vec3 normal(0.f);
    const float direction = (bodyB.position()[axis] >= bodyA.position()[axis]) ? 1.f : -1.f;
    normal[axis] = direction;

    const float invMassA = bodyA.mass() > 0.f ? 1.f / bodyA.mass() : 0.f;
    const float invMassB = bodyB.mass() > 0.f ? 1.f / bodyB.mass() : 0.f;
    const float invMassSum = invMassA + invMassB;
    if (invMassSum <= 1e-6f) {
        return false;
    }

    const float correctionMagnitude = std::max(0.f, minOverlap - 1e-4f);
    const glm::vec3 correction = normal * correctionMagnitude;
    bodyA.setPosition(bodyA.position() - correction * (invMassA / invMassSum));
    bodyB.setPosition(bodyB.position() + correction * (invMassB / invMassSum));

    glm::vec3 velA = bodyA.linearVelocity();
    glm::vec3 velB = bodyB.linearVelocity();
    const float relVelAlongNormal = glm::dot(velB - velA, normal);
    if (relVelAlongNormal < 0.f) {
        constexpr float restitution = 0.2f;
        const float impulseMag = -(1.f + restitution) * relVelAlongNormal / invMassSum;
        const glm::vec3 impulse = normal * impulseMag;

        velA -= impulse * invMassA;
        velB += impulse * invMassB;
        bodyA.setLinearVelocity(velA);
        bodyB.setLinearVelocity(velB);
    }

    bodyA.setAngularVelocity(bodyA.angularVelocity() * 0.98f);
    bodyB.setAngularVelocity(bodyB.angularVelocity() * 0.98f);
    return true;
}

inline void resolveRigidBodyPairCollisions(std::vector<RigidBody>& bodies) {
    if (bodies.size() < 2) {
        return;
    }

    for (int iter = 0; iter < 2; ++iter) {
        for (size_t i = 0; i < bodies.size(); ++i) {
            for (size_t j = i + 1; j < bodies.size(); ++j) {
                resolveRigidBodyPairCollision(bodies[i], bodies[j]);
            }
        }
    }
}

inline void resolveRigidBodyBoxCollision(RigidBody& body) {
    const auto& samples = body.worldSamples();
    if (samples.empty()) {
        return;
    }

    glm::vec3 minPoint(FLT_MAX);
    glm::vec3 maxPoint(-FLT_MAX);
    for (const auto& sample : samples) {
        minPoint = glm::min(minPoint, sample);
        maxPoint = glm::max(maxPoint, sample);
    }

    glm::vec3 correction(0.f);
    glm::vec3 velocity = body.linearVelocity();
    bool collided = false;

    for (int axis = 0; axis < 3; ++axis) {
        if (minPoint[axis] < Sim::params.boxMin[axis]) {
            correction[axis] += Sim::params.boxMin[axis] - minPoint[axis];
            if (velocity[axis] < 0.f) {
                velocity[axis] *= -Sim::params.rigidWallBounce;
            }
            collided = true;
        }
        if (maxPoint[axis] > Sim::params.boxMax[axis]) {
            correction[axis] -= maxPoint[axis] - Sim::params.boxMax[axis];
            if (velocity[axis] > 0.f) {
                velocity[axis] *= -Sim::params.rigidWallBounce;
            }
            collided = true;
        }
    }

    if (collided) {
        body.setPosition(body.position() + correction);
        body.setLinearVelocity(velocity);
        body.setAngularVelocity(body.angularVelocity() * Sim::params.rigidAngularWallDamping);
    }
}