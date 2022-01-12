#include <iostream>
#include <memory>

#include "bazier_traj/planner_parameter.h"
#include "bazier_traj/collision_map.h"
#include "bazier_traj/astar.h"
#include "region_segmentation.h"
#include "bezier_trajectory.h"

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


    Bernstein *bezier_ptr = new Bernstein(3, 9, 3);
    if(bezier_ptr -> setParam(3, 10, 3) == -1)
        ROS_ERROR(" The trajectory order is set beyond the library's scope, please re-set ");

    bezier_ptr->setParam(3, 12, 3);
    Eigen::MatrixXd MQM = bezier_ptr->getMQM()[9];
    Eigen::MatrixXd FM  = bezier_ptr->getFM()[9];
    Eigen::VectorXd C = bezier_ptr->getC()[9];
    Eigen::VectorXd Ca = bezier_ptr->getC_a()[9];
    Eigen::VectorXd Cv = bezier_ptr->getC_v()[9];
    Eigen::VectorXd Cj = bezier_ptr->getC_j()[9];

    TrajectoryGenerator *traj_ptr = new TrajectoryGenerator();
    ros::Rate rate(1);
    Eigen::MatrixXd pos = Eigen::MatrixXd::Zero(2,3);
    Eigen::MatrixXd vel = Eigen::MatrixXd::Zero(2,3);
    Eigen::MatrixXd acc = Eigen::MatrixXd::Zero(2,3);
    pos << 0, 0, 0,
        100, 100, 0;
    double MaxVel = 1.0;
    double MaxAcc = 3.0;
    double obj;
    Eigen::MatrixXd _bezier_coeff;

    while (ros::ok())
    {
        RegionSegmentation *region_ptr = new RegionSegmentation(map_ptr -> getLocalCollisionMap());
        region_ptr->advertiseInit(nh);
        path_search->importLocalMap(map_ptr->getLocalCollisionMap());
        path_search-> search(target);
        region_ptr->segmentation(path_search->getPath());
        traj_ptr->BezierPolyCoeffGeneration(region_ptr->getSegRes(), MQM, pos, vel, acc, MaxVel, MaxAcc, 10, 3, 0, obj, _bezier_coeff);
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}