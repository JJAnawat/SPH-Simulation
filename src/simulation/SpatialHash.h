#ifndef SPATIAL_HASH_H
#define SPATIAL_HASH_H

#include <glm/glm.hpp>
#include <unordered_map>
#include <unordered_set>
#include <vector>

struct SpatialHash{
    private:
        static constexpr size_t P1 = 73856093, P2 = 19349663, P3 = 83492791;
        float cell_size;
        std::unordered_map<size_t, std::vector<int>> grid;

        size_t hash(const glm::ivec3 &p) const;
        glm::ivec3 cell_pos(const glm::vec3 &real_pos) const;

    public:
        SpatialHash(float cs) : cell_size(cs) {}

        void insert(const std::vector<glm::vec3>& particles_pos);
        std::vector<int> query(const glm::vec3 &real_pos);
        
        void clear();
};

#endif