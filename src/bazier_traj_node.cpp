#include <iostream>
#include <memory>

#include "bazier_traj/planner_parameter.h"
#include "bazier_traj/collision_map.h"
#include "bazier_traj/astar.h"
#include "region_segmentation.h"

#include "vis.h"

using namespace planner;

int main(int argc, char** argv){
    ros::init(argc, argv, "bezier_traj_node");
    ros::NodeHandle nh("~");

    CollisionMap *map_ptr = new CollisionMap();
    map_ptr->advertisePub(nh);
    ros::Subscriber _map_sub = nh.subscribe("/map", 1000, &CollisionMap::rcvPointCloudCallback, map_ptr);//bool init
    //std::shared_ptr<AStar> path_search = std::make_shared<AStar>();
    AStar *path_search = new AStar();
    path_search->mapTest(nh);
    path_search -> setParam();
    path_search -> initLocalMap();
    Eigen::Vector3d target_pos;
    target_pos << 100.0, 100.0, 0.0;
    NodePtr target = path_search->setTarget(target_pos);


    ros::Rate rate(1);
    while (ros::ok())
    {
        RegionSegmentation *region_ptr = new RegionSegmentation(map_ptr -> getLocalCollisionMap());
        region_ptr->advertiseInit(nh);
        path_search->importLocalMap(map_ptr->getLocalCollisionMap());
        path_search-> search(target);
        region_ptr->segmentation(path_search->getPath());
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}