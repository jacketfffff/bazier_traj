#include "vis.h"

using namespace planner;

Visualization::Visualization(ros::NodeHandle &nh){
    _local_map_vis_pub = nh.advertise<visualization_msgs::MarkerArray>("collision_map", 10);
}

Visualization::~Visualization(){

}

void Visualization::mapVisualizationFromPtr(sdf_tools::CollisionMapGrid* map){
    _voxel_map.markers.clear();
        int id = 0;
        int idx_x = floor(PlannerParameter::_x_local_size * PlannerParameter::_inv_resolution);
        int idx_y = floor(PlannerParameter::_y_local_size * PlannerParameter::_inv_resolution);
        int idx_z = floor((PlannerParameter::_model_size_z + 1) * PlannerParameter::_inv_resolution);
        std::cout << idx_x << " ," << idx_y << " ," << idx_z << std::endl;
        for (int64_t i = 0; i < idx_x; i++){
            for (int64_t j = 0; j < idx_y; j++){
                for (int64_t k = 0; k < idx_z; k++){
                    double occupancy = map->Get(i, j, k).first.occupancy;
                    if(occupancy != 0.0){
                    std::cout << "occupancy = " << occupancy << std::endl;

                    }

        //             if (map_ -> Get(i, j, k).first.occupancy >= 0.5) {
        //             // int64_t a = 1, b = 1, c = 1;
        //             visualization_msgs::Marker box = voxelVis(i, j, k, id);
        //             _voxel_map.markers.push_back(box);
        //             ++id;
        //              }
                }
            }
        }
        // std::cout << _voxel_map.markers.size() << std::endl;
        // _local_map_vis_pub.publish(_voxel_map);
}

void Visualization::mapVisualizationFromMap(CollisionMap& map){
    auto map_ = map.getLocalCollisionMap();
    _voxel_map.markers.clear();
        int id = 0;
        int idx_x = floor(PlannerParameter::_x_local_size * PlannerParameter::_inv_resolution);
        int idx_y = floor(PlannerParameter::_y_local_size * PlannerParameter::_inv_resolution);
        int idx_z = floor((PlannerParameter::_model_size_z + 1) * PlannerParameter::_inv_resolution);
        std::cout << idx_x << " ," << idx_y << " ," << idx_z << std::endl;
        for (int64_t i = 0; i < idx_x; i++){
            for (int64_t j = 0; j < idx_y; j++){
                for (int64_t k = 0; k < idx_z; k++){
                    double occupancy = map_->Get(i, j, k).first.occupancy;
                    if(occupancy != 0.0){
                    std::cout << "occupancy = " << occupancy << std::endl;

                    }

        //             if (map_ -> Get(i, j, k).first.occupancy >= 0.5) {
        //             // int64_t a = 1, b = 1, c = 1;
        //             visualization_msgs::Marker box = voxelVis(i, j, k, id);
        //             _voxel_map.markers.push_back(box);
        //             ++id;
        //              }
                }
            }
        }
        // std::cout << _voxel_map.markers.size() << std::endl;
        // _local_map_vis_pub.publish(_voxel_map);
}

visualization_msgs::Marker Visualization::voxelVis(const int x, const int y, const int z, const int id){
        _voxel.header.frame_id = "world";
        _voxel.header.stamp = ros::Time::now();
        _voxel.ns = "map";
        _voxel.id = id;
        _voxel.type = visualization_msgs::Marker::CUBE;
        _voxel.action = visualization_msgs::Marker::ADD;

        _voxel.pose.position.x = (x + 0.5) * PlannerParameter::_resolution;
        _voxel.pose.position.y = (y + 0.5) * PlannerParameter::_resolution;
        _voxel.pose.position.z = (z + 0.5) * PlannerParameter::_resolution;
        _voxel.pose.orientation.x = 0.0;
        _voxel.pose.orientation.y = 0.0;
        _voxel.pose.orientation.z = 0.0;
        _voxel.pose.orientation.w = 1.0;

        _voxel.scale.x = PlannerParameter::_resolution;
        _voxel.scale.y = PlannerParameter::_resolution;
        _voxel.scale.z = PlannerParameter::_resolution;

        _voxel.color.a = 0.1;
        _voxel.color.r = 255.0;
        _voxel.color.g = 255.0;
        _voxel.color.b = 255.0;
        // std::cout << "test" << std::endl;
        return _voxel;
}
