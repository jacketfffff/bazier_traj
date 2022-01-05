#ifndef VIS_H
#define VIS_H

#include <ros/ros.h>
#include <ros/console.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <sdf_tools/collision_map.hpp>

#include "bazier_traj/collision_map.h"
#include "bazier_traj/planner_parameter.h"

namespace planner{
    class Visualization{
        public:
            Visualization(ros::NodeHandle &nh);
            ~Visualization();
            void mapVisualizationFromMap(CollisionMap& map);
            void mapVisualizationFromPtr(sdf_tools::CollisionMapGrid *map);
            visualization_msgs::Marker voxelVis(const int x, const int y, const int z, const int id);

        private:
            visualization_msgs::Marker _voxel;
            visualization_msgs::MarkerArray _voxel_map;
            ros::Publisher _local_map_vis_pub;
    };
};

#endif