#include "traj_node.h"

std::vector<pcl::PointXYZ> TrajectoryNode::pointInflate(pcl::PointXYZ &pt){
    int num = int(_cloud_margin * _inv_resolution);//cloud_margin 是膨胀参数
    int num_z = std::max(1, num / 2);
    std::vector<pcl::PointXYZ> infPts(20);
    pcl::PointXYZ pt_inf;
    for (int x = -num; x <= num; x++){
        for (int y = -num; y <= num; y++){
            for (int z = -num_z; z <= num_z; z++){
                pt_inf.x = pt.x + x * _resolution;
                pt_inf.y = pt.y + y * _resolution;
                pt_inf.z = pt.z + z * _resolution;

                infPts.push_back(pt_inf);
            }
        }
    }
    return infPts;
}

Eigen::Vector3d TrajectoryNode::getPosFromBezier(const Eigen::MatrixXd & polyCoeff, double t_now, int seg_now){
    Eigen::Vector3d ret = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd ctrl_now = polyCoeff.row(seg_now);
    int ctrl_num1D = polyCoeff.cols() / 3;

    for (int i = 0; i < 3; i++)
        for (int j = 0; j < ctrl_num1D; j++)
            ret(i) += _C(j) * ctrl_now(i * ctrl_num1D + j) * pow(t_now, j) * pow((1 - t_now), (_traj_order - j));
    return ret;
}

bool TrajectoryNode::checkCoordObs(Eigen::Vector3d & checkPt){
    if(collision_map -> Get( checkPt(0), checkPt(1), checkPt(2)).first.occupancy > 0.0)
        return true;
    return false;
}

bool TrajectoryNode::checkExecTraj(){
    if(_has_traj == false)
        return false;

    Eigen::Vector3d traj_pt;
    visualization_msgs::Marker _check_traj_vis, _stop_traj_vis;
    geometry_msgs::Point pt;

    _stop_traj_vis.header.stamp = _check_traj_vis.header.stamp = ros::Time::now();
    _stop_traj_vis.header.frame_id = _check_traj_vis.header.frame_id = "local_world";

    _check_traj_vis.ns = "trajectory/check_trajectory";
    _stop_traj_vis.ns = "trajectory/stop_trajectory";

    _stop_traj_vis.id = _check_traj_vis.id = 0;
    _stop_traj_vis.type = _check_traj_vis.type = visualization_msgs::Marker::SPHERE_LIST;
    _stop_traj_vis.action = _check_traj_vis.action = visualization_msgs::Marker::ADD;

    _stop_traj_vis.scale.x = 2.0 * _vis_traj_width;
    _stop_traj_vis.scale.y = 2.0 * _vis_traj_width;
    _stop_traj_vis.scale.z = 2.0 * _vis_traj_width;

    _check_traj_vis.scale.x = 1.5 * _vis_traj_width;
    _check_traj_vis.scale.y = 1.5 * _vis_traj_width;
    _check_traj_vis.scale.z = 1.5 * _vis_traj_width;

    _check_traj_vis.pose.orientation.x = 0.0;
    _check_traj_vis.pose.orientation.y = 0.0;
    _check_traj_vis.pose.orientation.z = 0.0;
    _check_traj_vis.pose.orientation.w = 1.0;

    _stop_traj_vis.color.r = 0.0;
    _stop_traj_vis.color.g = 1.0;
    _stop_traj_vis.color.b = 0.0;
    _stop_traj_vis.color.a = 1.0;

    _check_traj_vis.color.r = 0.0;
    _check_traj_vis.color.g = 0.0;
    _check_traj_vis.color.b = 1.0;
    _check_traj_vis.color.a = 1.0;

    double t_s = std::max(0.0, (_odom.header.stamp - _start_time).toSec());
    int idx;
    for (idx = 0; idx < _seg_num; idx++){
        if(t_s > _seg_time(idx) && idx + 1 < _seg_num)
            t_s -= _seg_time(idx);
        else
            break;
    }

    double duration = 0.0;
    double t_ss;
    for (int i = idx; i < _seg_num; i++){
        t_ss = (i == idx) ? t_s : 0.0;
        for (double t = t_ss; t < _seg_time(i); t += 0.01){
            double t_d = duration + t - t_ss;
            if(t_d > _check_horizon)
                break;
            traj_pt = getPosFromBezier(_bezier_coeff, t / _seg_time(i), i);
            pt.x = traj_pt(0) = _seg_time(i) * traj_pt(0);
            pt.y = traj_pt(1) = _seg_time(i) * traj_pt(1);
            pt.z = traj_pt(2) = _seg_time(i) * traj_pt(2);

            _check_traj_vis.points.push_back(pt);

            if(t_d <= _stop_horizon)
                _stop_traj_vis.points.push_back(pt);
            if(checkCoordObs(traj_pt)){
                ROS_WARN(" predicted collision time is %f ahead ", t_d);
                if(t_d <= _stop_horizon){
                    ROS_ERROR("emergency occurs in tine is %f ahead ", t_d);
                    _is_emerg = true;
                }
                _checkTraj_vis_pub.publish(_check_traj_vis);
                _stopTraj_vis_pub.publish(_stop_traj_vis);

                return true;
            }
        }
        duration += _seg_time(i) - t_ss;
    }
    _checkTraj_vis_pub.publish(_check_traj_vis);
    _stopTraj_vis_pub.publish(_stop_traj_vis);
}

void TrajectoryNode::rcvPointcloudCallback(const sensor_msgs::PointCloud2 & point_cloud){
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(point_cloud, cloud);

    if((int)cloud.points.size() == 0)
        return;

    delete collision_map_local;

    ros::Time time_1 = ros::Time::now();
    collision_map->RestMap();

    double local_c_x = (int)((_start_pt(0) - _x_local_size / 2.0) * _inv_resolution + 0.5) * _resolution;
    double local_c_y = (int)((_start_pt(1) - _y_local_size / 2.0) * _inv_resolution + 0.5) * _resolution;
    double local_c_z = (int)((_start_pt(2) - _z_local_size / 2.0) * _inv_resolution + 0.5) * _resolution;

    _local_origin << local_c_x, local_c_y, local_c_z;//按照区域索引
    Eigen::Translation3d origin_local_translation(_local_origin(0), _local_origin(1), _local_origin(2));
    Eigen::Quaterniond origin_local_rotation(1.0, 0.0, 0.0, 0.0);
    Eigen::Affine3d origin_local_transform = origin_local_translation * origin_local_rotation;

    double _buffer_size = 2 * _MAX_VEL;//额外加了2s的余量
    double _x_buffer_size = _x_local_size + _buffer_size;
    double _y_buffer_size = _y_local_size + _buffer_size;
    double _z_buffer_size = _z_local_size + _buffer_size;
    // sdf_tools::COLLISION_CELL _free_cell(0.0);

    collision_map_local = new sdf_tools::CollisionMapGrid(origin_local_transform, "local_world", _resolution, _x_buffer_size, _y_local_size, _z_local_size, _free_cell);

    std::vector<pcl::PointXYZ> inflatePts(20);
    pcl::PointCloud<pcl::PointXYZ> cloud_inflation;
    pcl::PointCloud<pcl::PointXYZ> cloud_local;

    for (int idx = 0; idx < (int)cloud.points.size(); idx++){
        auto mk = cloud.points[idx];
        pcl::PointXYZ pt(mk.x, mk.y, mk.z);
        if(fabs(pt.x - _start_pt(0)) > _x_local_size / 2.0 || fabs(pt.y - _start_pt(1)) > _y_local_size / 2.0 || fabs(pt.z - _start_pt(2)) > _z_local_size / 2.0)
            continue;
        cloud_local.push_back(pt);
        inflatePts = pointInflate(pt);
        for(int i = 0; i < (int)inflatePts.size(); i++){
            pcl::PointXYZ inf_pt = inflatePts[i];
            Eigen::Vector3d addPt(inf_pt.x, inf_pt.y, inf_pt.z);
            collision_map_local->Set3d(addPt, _obst_cell);
            collision_map->Set3d(addPt, _obst_cell);
            cloud_inflation.push_back(inf_pt);
        }
    }
    _has_map = true;

    cloud_inflation.width = cloud_inflation.points.size();
    cloud_inflation.height = 1;
    cloud_inflation.is_dense = true;
    cloud_inflation.header.frame_id = "local_world";

    cloud_local.width = cloud_local.points.size();
    cloud_local.height = 1;
    cloud_local.is_dense = true;
    cloud_local.header.frame_id = "local_world";

    sensor_msgs::PointCloud2 inflate_map, local_map;

    pcl::toROSMsg(cloud_local, local_map);
    pcl::toROSMsg(cloud_inflation, inflate_map);

    _local_map_vis_pub.publish(local_map);
    _inf_map_vis_pub.publish(inflate_map);

    //TODO 时间测试
    if(checkExecTraj() == true){
        trajectoryPlanning();//生成corridor和用贝塞尔曲线
    }

}