#ifndef ASTAR_H
#define ASTAR_H

#include <ros/ros.h>
#include <ros/console.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <sdf_tools/collision_map.hpp>
#include <algorithm>

#include "basic_a.h"


namespace planner{

    class AStar : public BasedAstar{
    public:
        AStar();
        ~AStar();

        //vis
        void mapVis(NodePtr*** _map);
        void mapTest(ros::NodeHandle &_nh);
        visualization_msgs::Marker voxelVis(const int x, const int y, const int z, const int id);
        void pathVis(const std::vector<NodePtr> &_path);
        visualization_msgs::Marker pathVoxelVis(const double x, const double y, const double z, const int id);

        // map
        // void localMap(sdf_tools::CollisionMapGrid *local_map, Eigen::Vector3d xyz_local);
        void importLocalMap(sdf_tools::CollisionMapGrid *_local_map);
        void resetLocalMap();
        void initLocalMap();


        // search
        double gScoreCalc(NodePtr _cur_node, NodePtr _pro_node) override;
        double fScoreCalc(NodePtr _cur_node) override;
        int search(NodePtr _end_pt) override;
        void clearNodeState();

        //collisionchecking
        bool collisionChecking(NodePtr ***_node_map, const Eigen::Vector3i pos_idx, const char symbol);

        // path
        void retrievePath(NodePtr cur_node) override;

        //target
        NodePtr setTarget(Eigen::Vector3d _end_pos);

        // init
        void setParam() override;
        // void init() override;

        // other
        Eigen::Vector3d index2coord(const Eigen::Vector3i &_index);
        //tag: local_map以body为中心，索引有正有负， coord2index 函数把index全部转化为正值处理
        Eigen::Vector3i coord2index(const Eigen::Vector3d &_coord);

        // interface
        NodePtr defineStartPt();
        NodePtr defineEndPt();
        void resetPath() override;
        std::vector<NodePtr> getPath();

    private:
        ros::Publisher local_map_vis_;
        ros::Publisher path_vis_;
        double tie_breaker_ = 1.0 + 1.0 / 10000; // A*工程经验，做一个小的增量
        // double map_size_x_, map_size_y_, map_size_z;

        //parameters for local_grid_map
        NodePtr*** GridNodeMap;
        Eigen::Vector3i map_idx_;
        Eigen::Vector3d map_size_;
        // double xl_{}, yl_{}, zl_{};
        double resolution_, inv_resolution_;
        visualization_msgs::MarkerArray voxel_map_;
        visualization_msgs::Marker voxel_;
        visualization_msgs::MarkerArray path_;
        visualization_msgs::Marker path_voxel_;
    };
}
#endif // ASTAR_H