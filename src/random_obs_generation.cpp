#include "bazier_traj/random_obs_generation.h"

MapGeneration::MapGeneration() {

}

MapGeneration::~MapGeneration() {

}

void MapGeneration::revOdomCallback(const nav_msgs::Odometry& odom) {

    if (odom.child_frame_id == "X" || odom.child_frame_id == "0")
        return;
    _has_odom = true;
    _state = {
            odom.pose.pose.position.x,
            odom.pose.pose.position.y,
            odom.pose.pose.position.z,
            odom.twist.twist.linear.x,
            odom.twist.twist.linear.y,
            odom.twist.twist.linear.z,
            0.0, 0.0, 0.0
    };
    //std::cout << " test odom get or not " << std::endl;
}

sensor_msgs::PointCloud2 MapGeneration::getCloudMap() {
    pcl::toROSMsg(cloudMap, globalMap_pcd);
    localMap_pcd.header.frame_id = "/world";
    return globalMap_pcd;
}

sensor_msgs::PointCloud2 MapGeneration::getLocalMap() {
    pcl::toROSMsg(localMap, localMap_pcd);
    localMap_pcd.header.frame_id = "world";
    return localMap_pcd;
}

void MapGeneration::randomMapGenerator() {
    pcl::PointXYZ pt_random;
    random_device rd;
    default_random_engine eng(rd());
    uniform_real_distribution<double> rand_x = uniform_real_distribution<double>(paramPtr -> _x_l, paramPtr -> _x_h);
    uniform_real_distribution<double> rand_y = uniform_real_distribution<double>(paramPtr -> _y_l, paramPtr -> _y_h);
    uniform_real_distribution<double> rand_w = uniform_real_distribution<double>(paramPtr -> _w_l, paramPtr -> _w_h);
    uniform_real_distribution<double> rand_h = uniform_real_distribution<double>(paramPtr -> _h_l, paramPtr -> _h_h);

    for(int i = 0; i< paramPtr -> _obs_num; i++){
        double x, y, w, h;
        x = rand_x(eng);
        y = rand_y(eng);
        w = rand_w(eng);

        if(sqrt(pow(x - paramPtr -> _init_x, 2) + pow(y - paramPtr -> _init_y, 2)) < 2.0)
            continue;
        x = floor(x / paramPtr -> _resolution) * paramPtr -> _resolution + paramPtr -> _resolution / 2.0;
        y = floor(y / paramPtr -> _resolution) * paramPtr -> _resolution + paramPtr -> _resolution / 2.0;
        int wid_num = ceil( w / paramPtr -> _resolution);
        for(int r = -wid_num/2.0; r < wid_num/2.0; r++) {
            for (int s = -wid_num / 2.0; s < wid_num / 2.0; s++) {
                h = rand_h(eng);
                int hei_num = ceil(h / paramPtr -> _resolution);
                for (int t = 0; t < hei_num; t++) {
                    pt_random.x = x + (r + 0.5) * paramPtr -> _resolution;
                    pt_random.y = y + (s + 0.5) * paramPtr -> _resolution;
                    pt_random.z = (t + 0.5) * paramPtr -> _resolution;
                    if(pt_random.x < 0.5 && pt_random.x > -0.5 &&
                       pt_random.y < 0.5 && pt_random.y > -0.5 &&
                       pt_random.z < 0.5 && pt_random.z > -0.5)
                        continue;
                    cloudMap.points.push_back(pt_random);
                }
            }
        }
    }
    cloudMap.width = cloudMap.points.size();
    cloudMap.height = 1;
    cloudMap.is_dense = true;

    kdTreeLocalMap.setInputCloud(cloudMap.makeShared());
    _map_ok = true;
}

void MapGeneration::sensedPoints() {
    if(!_map_ok || !_has_odom)
        return;
    pcl::PointXYZ searchPoint(_state[0], _state[1], _state[2]);
    pointIdxRadiusSearch.clear();
    pointRadiusSquareDistance.clear();
    localMap.clear();
    pcl::PointXYZ pt;
    if(isnan(searchPoint.x) || isnan(searchPoint.y) || isnan(searchPoint.z))
        return;
    if(kdTreeLocalMap.radiusSearch(searchPoint, paramPtr -> _sensing_range, pointIdxRadiusSearch, pointRadiusSquareDistance) > 0){
        for(size_t i = 0; i < pointIdxRadiusSearch.size(); i++){
            pt = cloudMap.points[pointIdxRadiusSearch[i]];
            localMap.points.push_back(pt);
        }
    }
    else{
        cout << "[Map Server] No obstacles" << endl;
        return;
    }
    localMap.width = localMap.points.size();
    localMap.height = 1;
    localMap.is_dense = true;
}