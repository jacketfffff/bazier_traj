#include <iostream>
#include <math.h>
#include "random"
#include <eigen3/Eigen/Dense>
#include <ros/ros.h>
#include <ros/console.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/Transform.h>
#include <tf2/LinearMath/Quaternion.h>

#include "bezier_traj/PositionCommand.h"



class OdomGeneartion{
public:
    OdomGeneartion(){

    }

    OdomGeneartion(double init_x, double init_y, double init_z){
        _init_x = init_x;
        _init_y = init_y;
        _init_z = init_z;
    }

    ~OdomGeneartion(){

    }

    void recPosCmdCallback(const bezier_traj::PositionCommand& cmd_vel){
        rcv_cmd = true;
        _cmd_vel = cmd_vel;
    }

    void pubOdom(){

        odom.header.stamp = ros::Time::now();
        odom.header.frame_id = "world";

        if(rcv_cmd){
            odom.pose.pose.position.x = _cmd_vel.position.x;
            odom.pose.pose.position.y = _cmd_vel.position.y;
            odom.pose.pose.position.z = _cmd_vel.position.z;

            odom.twist.twist.linear.x = _cmd_vel.velocity.x;
            odom.twist.twist.linear.y = _cmd_vel.velocity.y;
            odom.twist.twist.linear.z = _cmd_vel.velocity.z;

            odom.twist.twist.angular.x = _cmd_vel.acceleration.x;
            odom.twist.twist.angular.y = _cmd_vel.acceleration.y;
            odom.twist.twist.angular.z = _cmd_vel.acceleration.z;
        }
        else{
            odom.pose.pose.position.x = _init_x;
            odom.pose.pose.position.y = _init_y;
            odom.pose.pose.position.z = _init_z;

            odom.twist.twist.linear.x = 0.0;
            odom.twist.twist.linear.y = 0.0;
            odom.twist.twist.linear.z = 0.0;
        }

    }

private:

    double _init_x, _init_y, _init_z;
    bool rcv_cmd = false;
    bezier_traj::PositionCommand _cmd_vel;

public:
    ros::Subscriber _cmd_sub;
    ros::Publisher _odom_pub;
    nav_msgs::Odometry odom;
};

int main(int argc, char** argv){
    ros::init (argc, argv, "odom_generation");
    ros::NodeHandle nh("~");
    double init_x, init_y, init_z;
    nh.param("init_x", init_x, 0.0);
    nh.param("init_y", init_y, 0.0);
    nh.param("init_z", init_z, 0.0);

    // tf2_ros::TransformBroadcaster tfB;
    tf2_ros::TransformBroadcaster tfBroadcaster;
    geometry_msgs::TransformStamped transformStamped;
    transformStamped.header.frame_id = "world";
    transformStamped.child_frame_id = "robot";
    transformStamped.transform.translation.x = 0;
    transformStamped.transform.translation.y = 0;
    transformStamped.transform.translation.z = 0;
    tf2::Quaternion q;
    q.setRPY(0, 0, 0);
    transformStamped.transform.rotation.x = q.x();
    transformStamped.transform.rotation.y = q.y();
    transformStamped.transform.rotation.z = q.z();
    transformStamped.transform.rotation.w = q.w();

    OdomGeneartion odom_(init_x, init_y, init_z);
    odom_._cmd_sub = nh.subscribe("command", 1, &OdomGeneartion::recPosCmdCallback, &odom_);
    odom_._odom_pub = nh.advertise<nav_msgs::Odometry>("odometry", 1);

    ros::Rate rate(100);
    while(ros::ok()){
        odom_.pubOdom();
        odom_._odom_pub.publish(odom_.odom);
        transformStamped.header.stamp = ros::Time::now();
        tfBroadcaster.sendTransform(transformStamped);
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}