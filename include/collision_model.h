#ifndef COLLISION_MODEL_H
#define COLLISION_MODEL_H

#include <iostream>
#include <string>
#include <memory>
#include <eigen3/Eigen/Dense>


#include "bazier_traj/planner_parameter.h"

namespace planner{
enum shape
{
    cylinder,
    cube
};
// tag: 两种碰撞模型 radius 表示的是圆柱， x,y,z 表示立方体，这里的单位都是米,body系默认是雷达。
//            |z
//            |
//       _____|_______  坐标系在机器人顶端，x朝前，右手系
//     /      |_____/|_______y
//    /______/_____/ |
//    |     /      | |      碰撞检测范围： -model.x ~ model.x
//    |    /x      | |                   -model.y ~ model.y
//    |            | /                   -model.z ~ 0
//    |____________|/
class CollisionModel{
    public:
        CollisionModel();
        ~CollisionModel();
        Eigen::Vector3i setModel(const char _symbol);

    private:
        Eigen::Vector3i getModel(const shape _model);
        Eigen::Vector3i model_;
        shape shape_;
        double radius_;
        double x_size_, y_size_, z_size_;
        double resolution_, inv_resolution_;
};
typedef std::shared_ptr<CollisionModel> CollisionPtr;
// typedef CollisionModel *CollisionPtr;
}

#endif