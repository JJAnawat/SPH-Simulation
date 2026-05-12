#include "SpatialHash.h"

size_t SpatialHash::hash(const glm::ivec3 &p) const {
    const size_t out = 
        static_cast<size_t> (p.x * P1) ^ 
        static_cast<size_t> (p.y * P2) ^ 
        static_cast<size_t> (p.z * P3);
    return out;
}

glm::ivec3 SpatialHash::cell_pos(const glm::vec3 &real_pos) const {
    glm::ivec3 grid_index = {
        static_cast<int> (real_pos.x / cell_size),
        static_cast<int> (real_pos.y / cell_size),
        static_cast<int> (real_pos.z / cell_size)
    };
    return grid_index;
}

void SpatialHash::insert(const std::vector<glm::vec3>& particles_pos) {
    std::lock_guard<std::mutex> lock(*grid_mutex);
    grid.reserve(particles_pos.size());
    for(int i=0; i<particles_pos.size();i++){
        glm::ivec3 grid_idx = cell_pos(particles_pos[i]);
        size_t hash_idx = hash(grid_idx);
        grid[hash_idx].push_back(i);
    }
}

void SpatialHash::query(const glm::vec3 &real_pos, std::vector<int>& candidates) const {
    std::lock_guard<std::mutex> lock(*grid_mutex);
    glm::ivec3 grid_idx = cell_pos(real_pos);
    candidates.clear();
    candidates.reserve(64);
    
    for(int dx=-1; dx<=1; dx++){
        for(int dy=-1; dy<=1; dy++){
            for(int dz=-1; dz<=1; dz++){
                glm::ivec3 n_grid_idx = grid_idx + glm::ivec3(dx, dy, dz);
                size_t n_hash_idx = hash(n_grid_idx);
                
                auto it = grid.find(n_hash_idx);
                if(it != grid.end()) {
                    candidates.insert(candidates.end(), it->second.begin(), it->second.end());
                }
            }
        }
    }
}  

void SpatialHash::clear() {
    std::lock_guard<std::mutex> lock(*grid_mutex);
    grid.clear();
}