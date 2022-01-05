#include "bazier_traj/collision_map.h"
// todo: 在初始化voxel map的时候，同时把local_map的坐标系发布出去
namespace planner{

    CollisionMap::CollisionMap() :_free_cell(0.0), _obst_cell(1.0){
        _start_pt(0) = 0.0;
        _start_pt(1) = 0.0;
        _start_pt(2) = 0.0;
        double local_c_x = (int)((_start_pt(0) - PlannerParameter::_x_local_size / 2.0) * PlannerParameter::_inv_resolution + 0.5) * PlannerParameter::_resolution;
        double local_c_y = (int)((_start_pt(1) - PlannerParameter::_y_local_size / 2.0) * PlannerParameter::_inv_resolution + 0.5) * PlannerParameter::_resolution;
        double local_c_z = (int)((_start_pt(2) - PlannerParameter::_z_local_size / 2.0) * PlannerParameter::_inv_resolution + 0.5) * PlannerParameter::_resolution;
        //  #########
        //  #       # local_c_x : start_coord of local map
        //  #   *   # local_c_y : start_coord of local map
        //  #       # local_c_z : start_coord of local map
        //  #########
        _local_origin << local_c_x, local_c_y, local_c_z;
        // tag: 这里的Affine3d 定义的是lidar系到local_map的tf变换
        Eigen::Translation3d origin_local_translation(_local_origin(0), _local_origin(1), _local_origin(2));
        Eigen::Quaterniond origin_local_rotation(1.0, 0.0, 0.0, 0.0);
        Eigen::Affine3d origin_local_transform = origin_local_translation * origin_local_rotation;
        // buffer_size : local map size = local_size + 2s * max_vel
        // ?: 在计算旋转矩阵的过程中，local_c_x 对 start_pt 做了调整
        double _buffer_size = 2 * PlannerParameter::_MAX_VEL;
        double _x_buffer_size = PlannerParameter::_x_local_size + _buffer_size;
        double _y_buffer_size = PlannerParameter::_y_local_size + _buffer_size;
        double _z_buffer_size = PlannerParameter::_z_local_size ;
        local_collision_map = new sdf_tools::CollisionMapGrid(origin_local_transform, "world", PlannerParameter::_resolution, _x_buffer_size, _y_buffer_size, _z_buffer_size, _free_cell);

    }

    CollisionMap::~CollisionMap(){

    }

    // sdf_tools::CollisionMapGrid* CollisionMap::getCollisionMap(){
        // return collision_map;
    // }

    sdf_tools::CollisionMapGrid* CollisionMap::getLocalCollisionMap(){
        return local_collision_map;
    }

    std::vector<pcl::PointXYZ> CollisionMap::pointInflate(pcl::PointXYZ &pt){

        // tag: _cloud_margin 点云膨胀范围 如果 _cloud_margin = 0.2, _inv_resolution = 2, num = 0
        int num = int(PlannerParameter::_cloud_margin * PlannerParameter::_inv_resolution);
        int num_z = std::max(1, num / 2);
        std::vector<pcl::PointXYZ> infPts(20);
        pcl::PointXYZ pt_inf;
        for (int x = -num; x <= num; x++){
            for (int y = -num; y <= num; y++){
                for (int z = -num_z; z <= num_z; z++){
                    pt_inf.x = pt.x + x * PlannerParameter::_resolution;
                    pt_inf.y = pt.y + y * PlannerParameter::_resolution;
                    pt_inf.z = pt.z + z * PlannerParameter::_resolution;

                    infPts.push_back(pt_inf);
                }
            }
        }
        return infPts;
    }

    bool CollisionMap::ifHasMap(){
        return _has_map;
    }

    void CollisionMap::advertisePub(ros::NodeHandle &nh){
        _inf_map_vis_pub   = nh.advertise<sensor_msgs::PointCloud2>("vis_map_inflate", 1000);
        _local_map_vis_pub = nh.advertise<sensor_msgs::PointCloud2>("vis_map_local", 1000);
        _local_map_vis = nh.advertise<visualization_msgs::MarkerArray>("collision_map", 10);
    }

    void CollisionMap::mapVis(sdf_tools::CollisionMapGrid* map){
        _voxel_map.markers.clear();
        int id = 0;
        int idx_x = floor((PlannerParameter::_x_local_size) * PlannerParameter::_inv_resolution);
        int idx_y = floor((PlannerParameter::_y_local_size) * PlannerParameter::_inv_resolution);
        int idx_z = floor((PlannerParameter::_z_local_size) * PlannerParameter::_inv_resolution);

        for (int64_t i = 0; i < idx_x; i++){
            for (int64_t j = 0; j < idx_y; j++){
                for (int64_t k = 0; k < idx_z; k++){
                  if (map -> Get(i, j, k).first.occupancy >= 0.5) {
                    visualization_msgs::Marker box = voxelVis(i, j, k, id);
                    _voxel_map.markers.push_back(box);
                    ++id;
                     }
                }
            }
        }
        _local_map_vis.publish(_voxel_map);
    }

    visualization_msgs::Marker CollisionMap::voxelVis(const int x, const int y, const int z, const int id){
        _voxel.header.frame_id = "world";
        _voxel.header.stamp = ros::Time::now();
        _voxel.ns = "map";
        _voxel.id = id;
        _voxel.type = visualization_msgs::Marker::CUBE;
        _voxel.action = visualization_msgs::Marker::ADD;

        _voxel.pose.position.x = (x + 0.5) * PlannerParameter::_resolution - PlannerParameter::_x_local_size / 2.0 + 0.25;
        _voxel.pose.position.y = (y + 0.5) * PlannerParameter::_resolution - PlannerParameter::_y_local_size / 2.0 + 0.25;
        _voxel.pose.position.z = (z + 0.5) * PlannerParameter::_resolution - PlannerParameter::_z_local_size / 2.0 + 0.5;
        _voxel.pose.orientation.x = 0.0;
        _voxel.pose.orientation.y = 0.0;
        _voxel.pose.orientation.z = 0.0;
        _voxel.pose.orientation.w = 1.0;

        _voxel.scale.x = PlannerParameter::_resolution;
        _voxel.scale.y = PlannerParameter::_resolution;
        _voxel.scale.z = PlannerParameter::_resolution;

        _voxel.color.a = 0.8;
        _voxel.color.r = 0;
        _voxel.color.g = 1.0;
        _voxel.color.b = 0;
        return _voxel;
    }

    void CollisionMap::rcvPointCloudCallback(const sensor_msgs::PointCloud2 &pointcloud_map)
    {
        ros::Time time_1 = ros::Time::now();
        pcl::PointCloud<pcl::PointXYZ> cloud;
        pcl::fromROSMsg(pointcloud_map, cloud);

        if ((int)cloud.size() == 0)
            return;

        // boost::unique_lock<boost::mutex> lock(mutex);
        local_collision_map->RestMap();
        // ros::Time time_1 = ros::Time::now();

        std::vector<pcl::PointXYZ> inflatePts(20);
        pcl::PointCloud<pcl::PointXYZ> inflation_cloud;
        pcl::PointCloud<pcl::PointXYZ> local_cloud;

        for (int idx = 0; idx < (int)cloud.points.size(); idx++){

            auto mk = cloud.points[idx];
            pcl::PointXYZ pt(mk.x, mk.y, mk.z);
            //判断范围
            if(fabs(pt.x - _start_pt(0)) > PlannerParameter::_x_local_size / 2.0 ||
               fabs(pt.y - _start_pt(1)) > PlannerParameter::_y_local_size / 2.0 ||
               fabs(pt.z - _start_pt(2)) > PlannerParameter::_z_local_size )
                continue;

            if(pt.z > PlannerParameter::_z_local_size)
                continue;

            local_cloud.push_back(pt);
            inflatePts = pointInflate(pt);
            for (int i = 0; i < (int)inflatePts.size(); i++){
                pcl::PointXYZ inf_pt = inflatePts[i];
                if(inf_pt.x < 0.5 && inf_pt.x > -0.5 &&
                   inf_pt.y < 0.5 && inf_pt.y > -0.5 &&
                   inf_pt.z < 0.5 && inf_pt.z > -0.5)
                    continue;
                Eigen::Vector3d addPt(inf_pt.x, inf_pt.y, inf_pt.z);
                local_collision_map->Set3d(addPt, _obst_cell);
                // collision_map->Set3d(addPt, _obst_cell);

                inflation_cloud.push_back(inf_pt);
            }
        }
        mapVis(local_collision_map);

        _has_map = true;
        // todo: frame_id 需要考虑下
        inflation_cloud.width = inflation_cloud.points.size();
        inflation_cloud.height = 1;
        inflation_cloud.is_dense = true;
        inflation_cloud.header.frame_id = "world";

        local_cloud.width = local_cloud.points.size();
        local_cloud.height = 1;
        local_cloud.is_dense = true;
        local_cloud.header.frame_id = "world";

        sensor_msgs::PointCloud2 inflate_map, local_map;

        pcl::toROSMsg(inflation_cloud, inflate_map);
        pcl::toROSMsg(local_cloud, local_map);

        _inf_map_vis_pub.publish(inflate_map);
        _local_map_vis_pub.publish(local_map);
        ros::Time time_3 = ros::Time::now();
        double time_sec = (time_3 - time_1).toSec();
        // std::cout << "time = " << time_sec << std::endl;

    }
}