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
        const float volume = 8.f * halfExtents.x * halfExtents.y * halfExtents.z;
        body.m_density = volume > 0.f ? mass / volume : 0.f;

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
        const float volume = (4.f / 3.f) * glm::pi<float>() * radius * radius * radius;
        body.m_density = volume > 0.f ? mass / volume : 0.f;

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
    float density() const { return m_density; }
    const glm::mat3& localInertia() const { return m_localInertia; }
    const glm::mat3& inverseLocalInertia() const { return m_inverseLocalInertia; }
    const std::vector<glm::vec3>& localSamples() const { return m_localSamples; }
    const std::vector<glm::vec3>& worldSamples() const { return m_worldSamples; }
    size_t sampleCount() const { return m_worldSamples.size(); }
    
    const std::vector<glm::vec3>& meshVertices() const { return m_meshVertices; }
    const std::vector<unsigned int>& meshIndices() const { return m_meshIndices; }

    glm::mat3 getInverseInertiaWorld() const {
        const glm::mat3 rotation = glm::mat3_cast(m_orientation);
        return rotation * m_inverseLocalInertia * glm::transpose(rotation);
    }
    
    glm::mat4 getTransformMatrix() const {
        glm::mat4 transform = glm::translate(glm::mat4(1.f), m_position);
        transform *= glm::mat4_cast(m_orientation);
        return transform;
    }

private:
    void buildBoxMesh() {
        m_meshVertices.clear();
        m_meshIndices.clear();

        const float x = m_halfExtents.x;
        const float y = m_halfExtents.y;
        const float z = m_halfExtents.z;

        // Define 8 vertices of the box
        m_meshVertices = {
            {-x, -y, -z}, {x, -y, -z}, {x, y, -z}, {-x, y, -z},  // front face
            {-x, -y,  z}, {x, -y,  z}, {x, y,  z}, {-x, y,  z},  // back face
        };

        // Define 12 triangles (2 per face)
        m_meshIndices = {
            // front face (z = -z)
            0, 1, 2, 0, 2, 3,
            // back face (z = z)
            4, 6, 5, 4, 7, 6,
            // left face (x = -x)
            0, 3, 7, 0, 7, 4,
            // right face (x = x)
            1, 5, 6, 1, 6, 2,
            // top face (y = y)
            3, 2, 6, 3, 6, 7,
            // bottom face (y = -y)
            0, 4, 5, 0, 5, 1,
        };
    }

    void buildSphereMesh(float radius, int rings, int sectors) {
        m_meshVertices.clear();
        m_meshIndices.clear();

        rings = std::max(rings, 2);
        sectors = std::max(sectors, 3);

        // Generate vertices
        for (int ring = 0; ring <= rings; ++ring) {
            const float theta = glm::pi<float>() * static_cast<float>(ring) / static_cast<float>(rings);
            const float sinTheta = std::sin(theta);
            const float cosTheta = std::cos(theta);

            for (int sector = 0; sector <= sectors; ++sector) {
                const float phi = glm::two_pi<float>() * static_cast<float>(sector) / static_cast<float>(sectors);
                const float sinPhi = std::sin(phi);
                const float cosPhi = std::cos(phi);

                m_meshVertices.push_back({
                    radius * sinTheta * cosPhi,
                    radius * cosTheta,
                    radius * sinTheta * sinPhi
                });
            }
        }

        // Generate indices
        for (int ring = 0; ring < rings; ++ring) {
            for (int sector = 0; sector < sectors; ++sector) {
                const int a = ring * (sectors + 1) + sector;
                const int b = a + sectors + 1;

                m_meshIndices.push_back(a);
                m_meshIndices.push_back(b);
                m_meshIndices.push_back(a + 1);

                m_meshIndices.push_back(a + 1);
                m_meshIndices.push_back(b);
                m_meshIndices.push_back(b + 1);
            }
        }
    }

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

        // Generate samples on all 6 faces of the box
        for (int i = 0; i < samplesPerEdge; ++i) {
            for (int j = 0; j < samplesPerEdge; ++j) {
                const float u = last > 0 ? static_cast<float>(i) / static_cast<float>(last) : 0.f;
                const float v = last > 0 ? static_cast<float>(j) / static_cast<float>(last) : 0.f;
                
                const float x = glm::mix(-m_halfExtents.x, m_halfExtents.x, u);
                const float y = glm::mix(-m_halfExtents.y, m_halfExtents.y, v);
                const float z = glm::mix(-m_halfExtents.z, m_halfExtents.z, u);
                const float w = glm::mix(-m_halfExtents.z, m_halfExtents.z, v);

                // Front and back faces (z = ±halfExtent.z)
                addUniqueSample({x, y, -m_halfExtents.z});
                addUniqueSample({x, y,  m_halfExtents.z});
                
                // Left and right faces (x = ±halfExtent.x)
                addUniqueSample({-m_halfExtents.x, y, z});
                addUniqueSample({ m_halfExtents.x, y, z});
                
                // Top and bottom faces (y = ±halfExtent.y)
                addUniqueSample({x, -m_halfExtents.y, w});
                addUniqueSample({x,  m_halfExtents.y, w});
            }
        }
        
        buildBoxMesh();
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
        
        buildSphereMesh(radius, rings, sectors);
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
    float m_density = 0.f;
    glm::vec3 m_halfExtents{0.5f};
    glm::mat3 m_localInertia{1.f};
    glm::mat3 m_inverseLocalInertia{1.f};
    std::vector<glm::vec3> m_localSamples;
    std::vector<glm::vec3> m_worldSamples;
    std::vector<glm::vec3> m_meshVertices;
    std::vector<unsigned int> m_meshIndices;
};