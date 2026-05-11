#pragma once

#include <algorithm>
#include <cmath>
#include <glm/glm.hpp>
#include <glm/gtc/constants.hpp>
#include <glm/gtc/quaternion.hpp>
#include <vector>

class RigidBody {
public:
    static RigidBody createBox(
        const glm::vec3& position,
        const glm::vec3& halfExtents,
        float mass,
        int samplesPerEdge = 4
    ) {
        RigidBody body;
        body.m_position = position;
        body.m_mass = mass;
        body.m_halfExtents = halfExtents;

        const float x = halfExtents.x;
        const float y = halfExtents.y;
        const float z = halfExtents.z;

        const float ixx = (mass / 3.f) * (y * y + z * z);
        const float iyy = (mass / 3.f) * (x * x + z * z);
        const float izz = (mass / 3.f) * (x * x + y * y);

        body.m_localInertia = glm::mat3(0.f);
        body.m_localInertia[0][0] = ixx;
        body.m_localInertia[1][1] = iyy;
        body.m_localInertia[2][2] = izz;

        body.m_inverseLocalInertia = glm::mat3(0.f);
        body.m_inverseLocalInertia[0][0] = ixx > 0.f ? 1.f / ixx : 0.f;
        body.m_inverseLocalInertia[1][1] = iyy > 0.f ? 1.f / iyy : 0.f;
        body.m_inverseLocalInertia[2][2] = izz > 0.f ? 1.f / izz : 0.f;

        body.buildBoxSamples(samplesPerEdge);
        body.updateWorldSamples();
        return body;
    }

    static RigidBody createSphere(
        const glm::vec3& position,
        float radius,
        float mass,
        int rings = 8,
        int sectors = 16
    ) {
        RigidBody body;
        body.m_position = position;
        body.m_mass = mass;
        body.m_halfExtents = glm::vec3(radius);

        const float inertiaValue = 0.4f * mass * radius * radius;
        body.m_localInertia = glm::mat3(0.f);
        body.m_localInertia[0][0] = inertiaValue;
        body.m_localInertia[1][1] = inertiaValue;
        body.m_localInertia[2][2] = inertiaValue;

        const float inverseInertia = inertiaValue > 0.f ? 1.f / inertiaValue : 0.f;
        body.m_inverseLocalInertia = glm::mat3(0.f);
        body.m_inverseLocalInertia[0][0] = inverseInertia;
        body.m_inverseLocalInertia[1][1] = inverseInertia;
        body.m_inverseLocalInertia[2][2] = inverseInertia;

        body.buildSphereSamples(radius, rings, sectors);
        body.updateWorldSamples();
        return body;
    }

    void clearAccumulators() {
        m_force = glm::vec3(0.f);
        m_torque = glm::vec3(0.f);
    }

    void applyForce(const glm::vec3& force) {
        m_force += force;
    }

    void applyTorque(const glm::vec3& torque) {
        m_torque += torque;
    }

    void applyForceAtPoint(const glm::vec3& force, const glm::vec3& worldPoint) {
        m_force += force;
        m_torque += glm::cross(worldPoint - m_position, force);
    }

    void integrate(float dt) {
        if (m_mass <= 0.f || dt <= 0.f) {
            return;
        }

        const glm::vec3 linearAcceleration = m_force / m_mass;
        m_linearVelocity += linearAcceleration * dt;
        m_position += m_linearVelocity * dt;

        const glm::mat3 inverseWorldInertia = getInverseInertiaWorld();
        const glm::vec3 angularAcceleration = inverseWorldInertia * m_torque;
        m_angularVelocity += angularAcceleration * dt;

        const glm::quat omega(0.f, m_angularVelocity.x, m_angularVelocity.y, m_angularVelocity.z);
        m_orientation += 0.5f * omega * m_orientation * dt;
        m_orientation = glm::normalize(m_orientation);

        updateWorldSamples();
        clearAccumulators();
    }

    void setPosition(const glm::vec3& position) {
        m_position = position;
        updateWorldSamples();
    }

    void setOrientation(const glm::quat& orientation) {
        m_orientation = glm::normalize(orientation);
        updateWorldSamples();
    }

    void setLinearVelocity(const glm::vec3& velocity) {
        m_linearVelocity = velocity;
    }

    void setAngularVelocity(const glm::vec3& velocity) {
        m_angularVelocity = velocity;
    }

    const glm::vec3& position() const { return m_position; }
    const glm::quat& orientation() const { return m_orientation; }
    const glm::vec3& linearVelocity() const { return m_linearVelocity; }
    const glm::vec3& angularVelocity() const { return m_angularVelocity; }
    float mass() const { return m_mass; }
    const glm::mat3& localInertia() const { return m_localInertia; }
    const glm::mat3& inverseLocalInertia() const { return m_inverseLocalInertia; }
    const std::vector<glm::vec3>& localSamples() const { return m_localSamples; }
    const std::vector<glm::vec3>& worldSamples() const { return m_worldSamples; }
    size_t sampleCount() const { return m_worldSamples.size(); }

    glm::mat3 getInverseInertiaWorld() const {
        const glm::mat3 rotation = glm::mat3_cast(m_orientation);
        return rotation * m_inverseLocalInertia * glm::transpose(rotation);
    }

private:
    void buildBoxSamples(int samplesPerEdge) {
        m_localSamples.clear();

        samplesPerEdge = std::max(samplesPerEdge, 2);
        const int last = samplesPerEdge - 1;

        auto addUniqueSample = [this](const glm::vec3& sample) {
            constexpr float epsilon = 1e-5f;
            for (const auto& existing : m_localSamples) {
                if (glm::length(existing - sample) <= epsilon) {
                    return;
                }
            }
            m_localSamples.push_back(sample);
        };

        for (int i = 0; i < samplesPerEdge; ++i) {
            const float u = last > 0 ? static_cast<float>(i) / static_cast<float>(last) : 0.f;
            const float x = glm::mix(-m_halfExtents.x, m_halfExtents.x, u);

            for (int j = 0; j < samplesPerEdge; ++j) {
                const float v = last > 0 ? static_cast<float>(j) / static_cast<float>(last) : 0.f;
                const float y = glm::mix(-m_halfExtents.y, m_halfExtents.y, v);
                const float z = glm::mix(-m_halfExtents.z, m_halfExtents.z, v);

                addUniqueSample({x, y, -m_halfExtents.z});
                addUniqueSample({x, y,  m_halfExtents.z});
                addUniqueSample({x, -m_halfExtents.y, z});
                addUniqueSample({x,  m_halfExtents.y, z});
                addUniqueSample({-m_halfExtents.x, x, y});
                addUniqueSample({ m_halfExtents.x, x, y});
            }
        }
    }

    void buildSphereSamples(float radius, int rings, int sectors) {
        m_localSamples.clear();

        rings = std::max(rings, 2);
        sectors = std::max(sectors, 3);

        for (int ring = 0; ring <= rings; ++ring) {
            const float theta = glm::pi<float>() * static_cast<float>(ring) / static_cast<float>(rings);
            const float sinTheta = std::sin(theta);
            const float cosTheta = std::cos(theta);

            for (int sector = 0; sector < sectors; ++sector) {
                const float phi = glm::two_pi<float>() * static_cast<float>(sector) / static_cast<float>(sectors);
                const float sinPhi = std::sin(phi);
                const float cosPhi = std::cos(phi);

                m_localSamples.push_back({
                    radius * sinTheta * cosPhi,
                    radius * cosTheta,
                    radius * sinTheta * sinPhi
                });
            }
        }
    }

    void updateWorldSamples() {
        const glm::mat3 rotation = glm::mat3_cast(m_orientation);
        m_worldSamples.clear();
        m_worldSamples.reserve(m_localSamples.size());

        for (const auto& sample : m_localSamples) {
            m_worldSamples.push_back(m_position + rotation * sample);
        }
    }

    glm::vec3 m_position{0.f};
    glm::quat m_orientation{1.f, 0.f, 0.f, 0.f};
    glm::vec3 m_linearVelocity{0.f};
    glm::vec3 m_angularVelocity{0.f};
    glm::vec3 m_force{0.f};
    glm::vec3 m_torque{0.f};
    float m_mass = 1.f;
    glm::vec3 m_halfExtents{0.5f};
    glm::mat3 m_localInertia{1.f};
    glm::mat3 m_inverseLocalInertia{1.f};
    std::vector<glm::vec3> m_localSamples;
    std::vector<glm::vec3> m_worldSamples;
};