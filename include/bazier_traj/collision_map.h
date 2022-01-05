#ifndef COLLISION_MAP_H
#define COLLISION_MAP_H

#include <iostream>
#include <vector>
#include <boost/thread/mutex.hpp>

#include <eigen3/Eigen/Dense>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <sensor_msgs/PointCloud2.h>

#include <sdf_tools/collision_map.hpp>

#include "bazier_traj/planner_parameter.h"

namespace planner{
class CollisionMap{
    public:
        CollisionMap();
        ~CollisionMap();
        void rcvPointCloudCallback(const sensor_msgs::PointCloud2 &pointcloud_map);
        sdf_tools::CollisionMapGrid* getLocalCollisionMap();
        sdf_tools::CollisionMapGrid* getCollisionMap();
        bool ifHasMap();
        void advertisePub(ros::NodeHandle &nh);

        void mapVis(sdf_tools::CollisionMapGrid* map);
        visualization_msgs::Marker voxelVis(const int x, const int y, const int z, const int id);

    private:
        boost::mutex mutex;
        ros::Publisher _inf_map_vis_pub, _local_map_vis_pub;
        ros::Publisher _local_map_vis;
        visualization_msgs::Marker _voxel;
        visualization_msgs::MarkerArray _voxel_map;

        std::vector<pcl::PointXYZ> pointInflate(pcl::PointXYZ &pt);

        // sdf_tools::CollisionMapGrid *collision_map = new sdf_tools::CollisionMapGrid();
        // sdf_tools::CollisionMapGrid *local_collision_map = new sdf_tools::CollisionMapGrid();
        sdf_tools::CollisionMapGrid *local_collision_map;
        sdf_tools::COLLISION_CELL _free_cell;
        sdf_tools::COLLISION_CELL _obst_cell;
        Eigen::Vector3d _local_origin;
        visualization_msgs::MarkerArray map_vis;
        Eigen::Vector3d _start_pt, _start_vel; //_start_pt from odom

        bool _has_map = false;
};
};

#endif //COLLISION_MAP_H