#include <glad/glad.h>
#include "core/Renderer.h"
#include "core/Window.h"
#include "ui/DebugUI.h"
#include "simulation/RigidBody.h"
#include "simulation/SPHSolver.h"
#include "simulation/ParticleUtils.h"

#include <algorithm>
#include <cfloat>
#include <cmath>

static std::vector<RigidBody> createRigidBodies() {
    std::vector<RigidBody> bodies;
    bodies.emplace_back(RigidBody::createBox({-0.18f, 0.82f, 0.00f}, {0.14f, 0.08f, 0.10f}, 0.9f, 8));
    bodies.emplace_back(RigidBody::createBox({ 0.14f, 0.96f,-0.05f}, {0.10f, 0.06f, 0.08f}, 2.4f, 9));
    bodies.emplace_back(RigidBody::createSphere({ 0.02f, 0.74f, 0.10f}, 0.085f, 0.55f, 10, 20));
    return bodies;
}

static std::vector<Particle> createFluidParticles(int count) {
    count = std::max(count, 1);

    const int countPerAxis = std::max(1, static_cast<int>(std::ceil(std::cbrt(static_cast<double>(count)))));
    const int layerSize = countPerAxis * countPerAxis;
    const int layers = std::max(1, (count + layerSize - 1) / layerSize);
    const float spacing = 0.03f;
    const glm::vec3 origin = {
        -0.5f * spacing * static_cast<float>(countPerAxis - 1),
        -0.18f,
        -0.5f * spacing * static_cast<float>(countPerAxis - 1)
    };

    std::vector<Particle> particles;
    particles.reserve(count);

    for (int y = 0; y < layers && static_cast<int>(particles.size()) < count; ++y) {
        for (int z = 0; z < countPerAxis && static_cast<int>(particles.size()) < count; ++z) {
            for (int x = 0; x < countPerAxis && static_cast<int>(particles.size()) < count; ++x) {
                Particle p;
                p.mass = Sim::params.p_mass;
                p.position = origin + glm::vec3(static_cast<float>(x), static_cast<float>(y), static_cast<float>(z)) * spacing;
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

static bool resolveRigidBodyPairCollision(RigidBody& bodyA, RigidBody& bodyB) {
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

static void resolveRigidBodyPairCollisions(std::vector<RigidBody>& bodies) {
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

static void resolveRigidBodyBoxCollision(RigidBody& body) {
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

int main() {
    Window window(1280, 720, "SPH Fluid Simulation");
    
    DebugUI ui;
    ui.init(window.getHandle());
    
    Renderer renderer;
    renderer.init();

    auto particles = createFluidParticles(Sim::params.fluidParticleCount);
    SPHSolver solver(Sim::params.h, particles);
    std::vector<RigidBody> rigidBodies = createRigidBodies();

    Sim::params.showRigidBodies = true;
    Sim::params.rigidTwoWayCoupling = true;

    while (!window.shouldClose()) {
        window.pollEvents();

        if (Sim::params.resetRequested){
            particles = createFluidParticles(Sim::params.fluidParticleCount);
            solver = SPHSolver(Sim::params.h, particles);
            rigidBodies = createRigidBodies();
            Sim::params.resetRequested = false;
        }
        if (!Sim::params.paused) {
            solver.step(0.01f, rigidBodies);
            // Rigid bodies are integrated inside solver when two-way coupling is enabled.
            for (auto& body : rigidBodies) {
                resolveRigidBodyBoxCollision(body);
            }
            resolveRigidBodyPairCollisions(rigidBodies);
            for (auto& body : rigidBodies) {
                resolveRigidBodyBoxCollision(body);
            }
        }

        if (Sim::params.showRigidBodies)
            renderer.renderWithRigidBodies(solver.getParticles(), rigidBodies);
        else
            renderer.render(solver.getParticles());

        // ImGui
        ui.beginFrame();
        ui.render();
        ui.endFrame();

        window.swapBuffers();
    }

    renderer.shutdown();
    ui.shutdown();
    return 0;
}