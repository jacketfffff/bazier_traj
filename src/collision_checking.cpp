#include "collision_checking.h"

using namespace planner;

CollisionChecking::CollisionChecking(){

}

CollisionChecking::~CollisionChecking(){

}

bool CollisionChecking::obsCheck(const sdf_tools::CollisionMapGrid* collision_map, const Eigen::Vector3i &pos_idx, const char& symbol){
    Eigen::Vector3i map_size;
    if (symbol == CUBE_)
        map_size = _model -> setModel(CUBE_);
    else if (symbol == CYLINDER_)
        map_size = _model -> setModel(CYLINDER_);
    for (int64_t i = pos_idx(0) - map_size(0); i <= pos_idx(0) + map_size(0); ++i)
        for (int64_t j = pos_idx(1) - map_size(1); j <= pos_idx(1) + map_size(1); ++j)
            for (int64_t k = pos_idx(2) - map_size(2); k <= pos_idx(2); ++k){
                // double grid_occupancy = collision_map->Get(i, j, k).first.occupancy;
                if (collision_map->Get(i, j, k).first.occupancy > 0.5){
                    return false;
                }
            }
    return true;
}