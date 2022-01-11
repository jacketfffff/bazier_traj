#ifndef CUBE_SEGMENTATION_H
#define CUBE_SEGMENTATION_H

#include <iostream>
#include <vector>
#include <string>

#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/Dense>

#include <ros/ros.h>
#include <ros/console.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <sdf_tools/collision_map.hpp>

#include "bazier_traj/planner_parameter.h"
#include "bazier_traj/basic_a.h"

namespace planner{
    struct Cube{
        Eigen::Vector3d center_;
        Eigen::MatrixXd vertex_;
        double t_;

        Cube(){
            center_ = Eigen::Vector3d::Zero();
            vertex_ = Eigen::MatrixXd::Zero(8, 3);
        }
        ~Cube(){

        }
    };
    typedef Cube *cubePtr;

    class RegionSegmentation
    {
        private:
            bool isContain(cubePtr cube1, cubePtr cube2);
            cubePtr cubeGeneration(const Eigen::Vector3d _node_pos);
            std::pair<cubePtr, bool> cubeInflate(cubePtr _cube, cubePtr _lstcube);
            cubePtr setVertex(const Eigen::MatrixXd vertex_, const double resolution_);

            sdf_tools::CollisionMapGrid *collision_map_;
            std::vector<cubePtr> region_res_;
            cubePtr cubeMax_;
            std::vector<cubePtr> segmentation_res;

            ros::Publisher segmentation_res_vis_;
            visualization_msgs::Marker voxel_;
            visualization_msgs::MarkerArray segmentation_voxel_;

        public:
            RegionSegmentation();
            RegionSegmentation(sdf_tools::CollisionMapGrid *_collision_map);
            ~RegionSegmentation();
            void init();

            std::vector<cubePtr> getSegRes() {return region_res_;}
            void advertiseInit(ros::NodeHandle &_nh){
                segmentation_res_vis_ = _nh.advertise<visualization_msgs::MarkerArray>("segmentation_res", 10);
            }
            void segmentation(const std::vector<NodePtr> &_path_node_pool);
            void segmentationVis(const std::vector<cubePtr>& segmentation_res);
            visualization_msgs::Marker cubeVis(const double x, const double y, const double z, const double scale_x,
                                               const double scale_y, const double scale_z, const int id);

    };
}
#endif