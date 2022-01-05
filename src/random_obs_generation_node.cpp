#include "bazier_traj/random_obs_generation.h"
#include "iostream"
using namespace std;
// template<typename T, typename...Args>
// std::unique_ptr<T> make_unique(Args&&...args){
//     return std::unique_ptr<T>(new T(std::forward<Args>(args)...));
// }
unique_ptr<paramList> MapGeneration::paramPtr = make_unique<paramList>();
void setParam(ros::NodeHandle nh){
    nh.param("init_state_x", MapGeneration::paramPtr -> _init_x , 0.0);
    nh.param("init_state_y",MapGeneration::paramPtr -> _init_y,       0.0);
    nh.param("map/x_size", MapGeneration::paramPtr -> _x_size, 50.0);
    nh.param("map/y_size", MapGeneration::paramPtr -> _y_size, 50.0);
    nh.param("map/z_size", MapGeneration::paramPtr -> _z_size, 5.0 );
    nh.param("map/obs_num", MapGeneration::paramPtr -> _obs_num,  30);
    nh.param("map/resolution", MapGeneration::paramPtr -> _resolution, 0.2);
    nh.param("ObstacleShape/lower_rad", MapGeneration::paramPtr -> _w_l,   0.3);
    nh.param("ObstacleShape/upper_rad", MapGeneration::paramPtr -> _w_h,   0.8);
    nh.param("ObstacleShape/lower_hei", MapGeneration::paramPtr -> _h_l,   3.0);
    nh.param("ObstacleShape/upper_hei", MapGeneration::paramPtr -> _h_h,   7.0);

    nh.param("sensing/radius", MapGeneration::paramPtr -> _sensing_range, 5.0);
    nh.param("sensing/radius", MapGeneration::paramPtr -> _sense_rate, 10.0);
}

void putSensedPoints(MapGeneration& map, ros::Publisher& all_map, ros::Publisher& local_map){
    map.sensedPoints();
    static int i = 0;
    if(i < 10){
        all_map.publish(map.getCloudMap());
    }
    i++;
    if(i == INT_MAX) i--;
    //cout << "i = " << i << endl;
    local_map.publish(map.getLocalMap());
}

int main(int argc, char** argv){
    ros::init(argc, argv, "random_obs_generation_node");
    ros::NodeHandle _nh("~");

    ros::Publisher _local_map_pub = _nh.advertise<sensor_msgs::PointCloud2>("/random_forest", 10);
    ros::Publisher _all_map_pub   = _nh.advertise<sensor_msgs::PointCloud2>("/all_map", 10);

    setParam(_nh);
    MapGeneration _map;

    ros::Subscriber _odom_sub = _nh.subscribe("/odom_generation/odometry", 50, &MapGeneration::revOdomCallback, &_map);

    setParam(_nh);

    MapGeneration::paramPtr -> _x_l = - MapGeneration::paramPtr -> _x_size / 2.0;
    MapGeneration::paramPtr -> _x_h = + MapGeneration::paramPtr -> _x_size / 2.0;
    MapGeneration::paramPtr -> _y_l = - MapGeneration::paramPtr -> _y_size / 2.0;
    MapGeneration::paramPtr -> _y_h = + MapGeneration::paramPtr -> _y_size / 2.0;

    MapGeneration::paramPtr -> _obs_num = min( MapGeneration::paramPtr -> _obs_num, (int) MapGeneration::paramPtr -> _x_size * 10);
    MapGeneration::paramPtr -> _z_limit =  MapGeneration::paramPtr -> _z_size;

    _map.randomMapGenerator();
    _nh.param("sensing/radius", MapGeneration::paramPtr -> _sense_rate, 10.0);
    ros::Rate loop_rate( MapGeneration::paramPtr -> _sense_rate);

    while(ros::ok()){
        putSensedPoints(_map, _all_map_pub, _local_map_pub);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}