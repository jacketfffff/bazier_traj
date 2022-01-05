#include "collision_model.h"


using namespace planner;

CollisionModel::CollisionModel(){
    radius_ = PlannerParameter::_radius;
    x_size_ = PlannerParameter::_model_size_x;
    y_size_ = PlannerParameter::_model_size_y;
    z_size_ = PlannerParameter::_model_size_z;
    inv_resolution_ = PlannerParameter::_inv_resolution;
    resolution_ = PlannerParameter::_resolution;
}

CollisionModel::~CollisionModel(){

}

Eigen::Vector3i CollisionModel::setModel(const char _symbol){
    if(_symbol == CUBE_)
        shape_ = cube;
    else if(_symbol == CYLINDER_)
        shape_ = cylinder;
    model_ = getModel(shape_);
    return model_;
}

Eigen::Vector3i CollisionModel::getModel(const shape _model){
    Eigen::Vector3i model_size;
    int x, y, z;
    switch (_model)
    {
    case cylinder:
        z = ceil(z_size_ * inv_resolution_);
        x = ceil(radius_ * inv_resolution_);
        y = x;
        model_size << x, y, z;
        break;
    case cube:
        x = ceil(x_size_ * inv_resolution_ / 2);
        y = ceil(y_size_ * inv_resolution_ / 2);
        z = ceil(z_size_ * inv_resolution_);
        model_size << x, y, z;
        break;
    default:
        break;
    }

    return model_size;
}