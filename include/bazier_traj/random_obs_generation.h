#include "iostream"
#include "string"
#include "time.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/search/kdtree.h"
#include "math.h"
#include "random"
#include "memory"
#include "sensor_msgs/PointCloud2.h"
#include "nav_msgs/Odometry.h"
#include "boost/move/unique_ptr.hpp"
#include "boost/move/make_unique.hpp"
using namespace std;
// param struct
struct paramList{
    paramList(){}
    int _obs_num{};
    double _x_l{}, _x_h{}, _y_l{}, _y_h{}, _w_l{}, _w_h{}, _h_l{}, _h_h{};
    double _z_limit{}, _sensing_range{}, _resolution{}, _sense_rate{}, _init_x{}, _init_y{};
    double _x_size{}, _y_size{}, _z_size{};
};

class MapGeneration{
public:
    MapGeneration();
    ~MapGeneration();

//    static unique_ptr<paramList> paramPtr;
    static unique_ptr<paramList> paramPtr;
    void revOdomCallback(const nav_msgs::Odometry& odom);
    void randomMapGenerator();
    void sensedPoints();
    sensor_msgs::PointCloud2 getCloudMap();
    sensor_msgs::PointCloud2 getLocalMap();

private:
    pcl::search::KdTree<pcl::PointXYZ> kdTreeLocalMap;
    vector<int> pointIdxRadiusSearch;
    vector<float> pointRadiusSquareDistance;
    vector<double> _state;
    bool _map_ok{false}, _has_odom{false};
    sensor_msgs::PointCloud2 localMap_pcd;
    sensor_msgs::PointCloud2 globalMap_pcd;
    pcl::PointCloud<pcl::PointXYZ> cloudMap, localMap;
};