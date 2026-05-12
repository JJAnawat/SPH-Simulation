#ifndef SPATIAL_HASH_H
#define SPATIAL_HASH_H

#include <glm/glm.hpp>
#include <unordered_map>
#include <vector>
#include <mutex>
#include <memory>

struct SpatialHash{
    private:
        static constexpr size_t P1 = 73856093, P2 = 19349663, P3 = 83492791;
        float cell_size;
        std::unordered_map<size_t, std::vector<int>> grid;
        mutable std::unique_ptr<std::mutex> grid_mutex;

        size_t hash(const glm::ivec3 &p) const;
        glm::ivec3 cell_pos(const glm::vec3 &real_pos) const;

    public:
        SpatialHash(float cs) : cell_size(cs), grid_mutex(std::make_unique<std::mutex>()) {}

        void insert(const std::vector<glm::vec3>& particles_pos);
        void query(const glm::vec3 &real_pos, std::vector<int>& candidates) const;
        
        void clear();
};

#endif