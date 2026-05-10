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

void SpatialHash::insert(const glm::vec3 &real_pos, int idx) {
    glm::ivec3 grid_idx = cell_pos(real_pos);
    size_t hash_idx = hash(grid_idx);
    grid[hash_idx].push_back(idx);
}

std::vector<int> SpatialHash::query(const glm::vec3 &real_pos) {
    glm::ivec3 grid_idx = cell_pos(real_pos);
    size_t hash_idx = hash(grid_idx);
    
    std::vector<int> candidates;
    std::unordered_set<size_t> vis;
    
    for(int dx=-1; dx<=1; dx++){
        for(int dy=-1; dy<=1; dy++){
            for(int dz=-1; dz<=1; dz++){
                glm::ivec3 n_grid_idx = grid_idx + glm::ivec3(dx, dy, dz);
                size_t n_hash_idx = hash(n_grid_idx);
                
                auto it = grid.find(n_hash_idx);
                if(it != grid.end())
                    for(int id: it->second)
                        if(vis.insert(id).second)
                            candidates.push_back(id);
            }
        }
    }

    return candidates;
}  

void SpatialHash::clear() {
    grid.clear();
}