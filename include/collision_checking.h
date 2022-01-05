#ifndef COLLISION_CHECKING_H
#define COLLISION_CHECKING_H

#include <iostream>

#include "collision_model.h"
#include "bazier_traj/basic_a.h"

namespace planner{
class CollisionChecking{
    public:
        CollisionChecking();
        ~CollisionChecking();
        bool obsCheck(const sdf_tools::CollisionMapGrid* collision_map, const Eigen::Vector3i &pos_idx, const char& symbol);

    private:
        CollisionModel* _model;
};
}





#endif