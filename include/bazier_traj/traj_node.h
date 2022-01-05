#ifndef TRAJ_NODE_H
#define TRAJ_NODE_H

#include <ros/ros.h>
#include <ros/console.h>

#include <iostream>
#include <fstream>
#include <math.h>
#include <random>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/search/kdtree.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sdf_tools/collision_map.hpp>
#include <eigen3/Eigen/Dense>

#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <memory>

class TrajectoryNode{
public:
    TrajectoryNode():_free_cell(0.0),_obst_cell(0.0){}
    void rcvPointcloudCallback(const sensor_msgs::PointCloud2 &point_cloud);
    void rcvOdomCallback(const nav_msgs::Odometry &odom);
    void trajectoryPlanning();
    Eigen::Vector3d getPosFromBezier(const Eigen::MatrixXd &polyCoeff, double t_now, int seg_now);
    bool checkCoordObs(Eigen::Vector3d &checkPt);

private:
    // std::shared_ptr<sdf_tools::CollisionMapGrid> collision_map = std::make_shared<sdf_tools::CollisionMapGrid>();
    // std::shared_ptr<sdf_tools::CollisionMapGrid> collision_map_local = std::make_shared<sdf_tools::CollisionMapGrid>();
    sdf_tools::CollisionMapGrid* collision_map = new sdf_tools::CollisionMapGrid();
    sdf_tools::CollisionMapGrid *collision_map_local = new sdf_tools::CollisionMapGrid();
    ros::NodeHandle nh_;
    ros::Subscriber _map_sub, _pts_sub, _odom_pub;
    ros::Publisher _inf_map_vis_pub, _local_map_vis_pub;

    Eigen::Vector3d _start_pt, _start_vel, _start_acc, _end_pt;//机器人状态
    Eigen::Vector3d _local_origin;
    double _x_size, _y_size, _z_size, _x_local_size, _y_local_size, _z_local_size; //地图的size
    double _inv_resolution, _resolution;//地图分辨率

    double _MAX_VEL;
    double _cloud_margin;
    sdf_tools::COLLISION_CELL _free_cell;
    sdf_tools::COLLISION_CELL _obst_cell;
    std::vector<pcl::PointXYZ> pointInflate(pcl::PointXYZ &pt);
    bool checkExecTraj();
    bool _has_map{false};
    bool _has_traj{false};
    //TODO simulation param
    double _vis_traj_width;
    nav_msgs::Odometry _odom;
    ros::Time _start_time{ros::TIME_MAX};
    int _seg_num;
    Eigen::VectorXd _seg_time;
    double _check_horizon, _stop_horizon;
    Eigen::MatrixXd _bezier_coeff;
    bool _is_emerg;
    Eigen::VectorXd _C;
    ros::Publisher _checkTraj_vis_pub, _stopTraj_vis_pub;
    int _traj_order;
};

#endif