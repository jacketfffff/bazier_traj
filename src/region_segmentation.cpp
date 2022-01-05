#include "region_segmentation.h"

namespace planner{
    RegionSegmentation::RegionSegmentation(){

    }

    RegionSegmentation::RegionSegmentation(sdf_tools::CollisionMapGrid *_collision_map){
        collision_map_ = _collision_map;
    }

    RegionSegmentation::~RegionSegmentation(){

    }

    cubePtr RegionSegmentation::cubeGeneration(const Eigen::Vector3d _node_pos){
        //todo: 从里程计获得世界系的坐标，或者读tf做个变换，这里的_node_pos指的是localmap下的坐标

        cubePtr cube = new Cube();
        cube->center_ = _node_pos;
        double x_u = _node_pos(0);
        double x_l = _node_pos(0);
        double y_u = _node_pos(1);
        double y_l = _node_pos(1);
        double z_u = _node_pos(2);
        double z_l = _node_pos(2);

        cube -> vertex_.row(0) = Eigen::Vector3d(x_u, y_l, z_u);
        cube -> vertex_.row(1) = Eigen::Vector3d(x_u, y_u, z_u);
        cube -> vertex_.row(2) = Eigen::Vector3d(x_l, y_u, z_u);
        cube -> vertex_.row(3) = Eigen::Vector3d(x_l, y_l, z_u);
        cube -> vertex_.row(4) = Eigen::Vector3d(x_u, y_l, z_l);
        cube -> vertex_.row(5) = Eigen::Vector3d(x_u, y_u, z_l);
        cube -> vertex_.row(6) = Eigen::Vector3d(x_l, y_u, z_l);
        cube -> vertex_.row(7) = Eigen::Vector3d(x_l, y_l, z_l);
        return cube;
    }

    cubePtr RegionSegmentation::setVertex(const Eigen::MatrixXd _vertex, const double _resolution){
            cubeMax_ ->vertex_ = _vertex;
            cubeMax_ ->vertex_(0,1) -= _resolution / 2.0;
            cubeMax_ ->vertex_(3,1) -= _resolution / 2.0;
            cubeMax_ ->vertex_(4,1) -= _resolution / 2.0;
            cubeMax_ ->vertex_(7,1) -= _resolution / 2.0;

            cubeMax_ ->vertex_(1,1) += _resolution / 2.0;
            cubeMax_ ->vertex_(2,1) += _resolution / 2.0;
            cubeMax_ ->vertex_(5,1) += _resolution / 2.0;
            cubeMax_ ->vertex_(6,1) += _resolution / 2.0;

            cubeMax_ ->vertex_(0,0) += _resolution / 2.0;
            cubeMax_ ->vertex_(1,0) += _resolution / 2.0;
            cubeMax_ ->vertex_(4,0) += _resolution / 2.0;
            cubeMax_ ->vertex_(5,0) += _resolution / 2.0;

            cubeMax_ ->vertex_(2,0) -= _resolution / 2.0;
            cubeMax_ ->vertex_(3,0) -= _resolution / 2.0;
            cubeMax_ ->vertex_(6,0) -= _resolution / 2.0;
            cubeMax_ ->vertex_(7,0) -= _resolution / 2.0;

            cubeMax_ ->vertex_(0,2) += _resolution / 2.0;
            cubeMax_ ->vertex_(1,2) += _resolution / 2.0;
            cubeMax_ ->vertex_(2,2) += _resolution / 2.0;
            cubeMax_ ->vertex_(3,2) += _resolution / 2.0;

            cubeMax_ ->vertex_(4,2) -= _resolution / 2.0;
            cubeMax_ ->vertex_(5,2) -= _resolution / 2.0;
            cubeMax_ ->vertex_(6,2) -= _resolution / 2.0;
            cubeMax_ ->vertex_(7,2) -= _resolution / 2.0;

            return cubeMax_;
    }

    bool RegionSegmentation::isContain(cubePtr _cube1, cubePtr _cube2){
        if( _cube1 -> vertex_(0, 0) >= _cube2 -> vertex_(0, 0) && _cube1 -> vertex_(0, 1) <= _cube2 -> vertex_(0, 1) && _cube1 -> vertex_(0, 2) >= _cube2 -> vertex_(0, 2) &&
            _cube1 -> vertex_(6, 0) <= _cube2 -> vertex_(6, 0) && _cube1 -> vertex_(6, 1) >= _cube2 -> vertex_(6, 1) && _cube1 -> vertex_(6, 2) <= _cube2 -> vertex_(6, 2)  )
            return true;
        else
            return false;
    }

    std::pair<cubePtr, bool> RegionSegmentation::cubeInflate(cubePtr _cube, cubePtr _lstcube)
    {
        cubeMax_ = _cube;
        Eigen::MatrixXi vertex_idx(8, 3);
        for (int i = 0; i < 8; i++){
            //tag: 这里的cube的中心点坐标放在local map坐标系下面
            double coord_x = std::max(std::min(_cube->vertex_(i, 0), PlannerParameter::_x_local_size / 2), -PlannerParameter::_x_local_size / 2);
            double coord_y = std::max(std::min(_cube->vertex_(i, 1), PlannerParameter::_y_local_size / 2), -PlannerParameter::_y_local_size / 2);
            double coord_z = std::max(std::min(_cube->vertex_(i, 2), PlannerParameter::_z_local_size), 0.0);
            Eigen::Vector3d coord(coord_x, coord_y, coord_z);

            Eigen::Vector3i pt_idx = collision_map_->LocationToGridIndex(coord);

            if(collision_map_ -> Get((int64_t)pt_idx(0), (int64_t)pt_idx(1), (int64_t)pt_idx(2)).first.occupancy > 0.5){
                ROS_ERROR("path has node in obstacles");
                return std::make_pair(cubeMax_, false);
            }
            vertex_idx.row(i) = pt_idx;
        }
        int idx, idy, idz;
            /*
               P4------------P3
               /|           /|              ^
              / |          / |              | z
            P1--|---------P2 |              |
             |  P8--------|--p7             |
             | /          | /               /--------> y
             |/           |/               /
            P5------------P6              / x
        */

        // Y- now is the left side : (p1 -- p4 -- p8 -- p5) face sweep
        bool collide;
        Eigen::MatrixXi vertex_idx_lst = vertex_idx;
        int iter = 0;
        while(iter < PlannerParameter::_max_inflate_iter){
            // inflate y
            collide = false;
            int y_lo = std::max(0, vertex_idx(0,1) - PlannerParameter::_step_length);
            int y_up = std::min(PlannerParameter::_max_y_id, vertex_idx(1, 1) + PlannerParameter::_step_length);
            for (idy = vertex_idx(0, 1); idy >= y_lo; idy--){
                if(collide == true)
                    break;
                for (idx = vertex_idx(0, 0); idx >= vertex_idx(3, 0); idx--){
                    if(collide == true)
                        break;
                    for (idz = vertex_idx(0, 2); idz >= vertex_idx(4, 2); idz--){
                        if(collision_map_ -> Get((int64_t)idx, (int64_t)idy, (int64_t)idz).first.occupancy > 0.5){
                            collide = true;
                            break;
                        }
                    }
                }
            }
            if(collide){
                // step_length_ = 2, y_lo = vertex_idx(0, 1) - _step_length; tag: 搞自己
                vertex_idx(0, 1) = std::min(idy + 2, vertex_idx(0, 1));
                vertex_idx(3, 1) = std::min(idy + 2, vertex_idx(3, 1));
                vertex_idx(7, 1) = std::min(idy + 2, vertex_idx(7, 1));
                vertex_idx(4, 1) = std::min(idy + 2, vertex_idx(4, 1));
            }
            else
                vertex_idx(0, 1) = vertex_idx(3, 1) = vertex_idx(7, 1) = vertex_idx(4, 1) = idy + 1;

            collide = false;
            for(idy = vertex_idx(1, 1); idy <= y_up; idy++){
                if(collide == true)
                    break;
                for (idx = vertex_idx(1, 0); idx >= vertex_idx(2, 0); idx--){
                    if(collide == true)
                        break;
                    for (idz = vertex_idx(1, 2); idz >= vertex_idx(5, 2); idz--){
                        if(collision_map_ -> Get((int64_t)idx, (int64_t)idy, (int64_t)idz).first.occupancy > 0.5){
                            collide = true;
                            break;
                        }
                    }
                }
            }
            if(collide){
                vertex_idx(1, 1) = std::max(idy - 2, vertex_idx(1, 1));
                vertex_idx(2, 1) = std::max(idy - 2, vertex_idx(2, 1));
                vertex_idx(6, 1) = std::max(idy - 2, vertex_idx(6, 1));
                vertex_idx(5, 1) = std::max(idy - 2, vertex_idx(5, 1));
            }
            else
                vertex_idx(1, 1) = vertex_idx(2, 1) = vertex_idx(6, 1) = vertex_idx(5, 1) = idy - 1;

            // inflate x
            int x_lo = std::max(0, vertex_idx(3, 0) - PlannerParameter::_step_length);
            int x_up = std::max(PlannerParameter::_max_x_id, vertex_idx(0, 0) + PlannerParameter::_step_length);

            collide = false;
            for (idx = vertex_idx(0, 0); idx <= x_up; idx++){
                if(collide == true)
                    break;
                for (idy = vertex_idx(0, 1); idy <= vertex_idx(1, 1); idy++){
                    if(collide == true)
                        break;
                    for (idz = vertex_idx(0, 2); idz >= vertex_idx(4, 2); idz--){
                        if(collision_map_ -> Get((int64_t)idx, (int64_t)idy, (int64_t)idz).first.occupancy > 0.5){
                            collide = true;
                            break;
                        }
                    }
                }
            }
            if(collide){
                vertex_idx(0, 0) = std::max(idx - 2, vertex_idx(0, 0));
                vertex_idx(1, 0) = std::max(idx - 2, vertex_idx(1, 0));
                vertex_idx(5, 0) = std::max(idx - 2, vertex_idx(5, 0));
                vertex_idx(4, 0) = std::max(idx - 2, vertex_idx(4, 0));
            }
            else
                vertex_idx(0, 0) = vertex_idx(1, 0) = vertex_idx(5, 0) = vertex_idx(4, 0) = idx - 1;

            collide = false;
            for (idx = vertex_idx(3, 0); idx >= x_lo; idx--){
                if(collide == true)
                    break;
                for (idy = vertex_idx(3, 1); idy <= vertex_idx(2, 1); idy++){
                    if(collide == true)
                        break;
                    for (idz = vertex_idx(3, 2); idz >= vertex_idx(7, 2); idz--){
                        if(collision_map_ -> Get((int64_t)idx, (int64_t)idy, (int64_t)idz).first.occupancy > 0.5){
                            collide = true;
                            break;
                        }
                    }
                }
            }
            if(collide){
                vertex_idx(3, 0) = std::min(idx + 2, vertex_idx(3, 0));
                vertex_idx(2, 0) = std::min(idx + 2, vertex_idx(2, 0));
                vertex_idx(6, 0) = std::min(idx + 2, vertex_idx(6, 0));
                vertex_idx(7, 0) = std::min(idx + 2, vertex_idx(7, 0));
            }
            else
                vertex_idx(3, 0) = vertex_idx(2, 0) = vertex_idx(6, 0) = vertex_idx(7, 0) = idx + 1;

            //inflate z
            collide = false;
            int z_lo = std::max(0, vertex_idx(4, 2) - PlannerParameter::_step_length);
            int z_up = std::max(PlannerParameter::_max_z_id, vertex_idx(0, 2) + PlannerParameter::_step_length);
            for (idz = vertex_idx(0, 2); idz <= z_up; idz++){
                if(collide == true)
                    break;
                for (idy = vertex_idx(0, 1); idy <= vertex_idx(1, 1); idy++){
                    if(collide == true)
                        break;
                    for (idx = vertex_idx(0, 0); idx >= vertex_idx(3, 0); idx--){
                        if(collision_map_ -> Get((int64_t)idx, (int64_t)idy, (int64_t)idz).first.occupancy > 0.5){
                            collide = true;
                            break;
                        }
                    }
                }
            }
            if(collide){
                vertex_idx(0, 2) = std::max(idz - 2, vertex_idx(0, 2));
                vertex_idx(1, 2) = std::max(idz - 2, vertex_idx(1, 2));
                vertex_idx(2, 2) = std::max(idz - 2, vertex_idx(2, 2));
                vertex_idx(3, 2) = std::max(idz - 2, vertex_idx(3, 2));
            }
            else
                vertex_idx(0, 2) = vertex_idx(1, 2) = vertex_idx(2, 2) = vertex_idx(3, 2) = idz - 1;

            if(vertex_idx_lst == vertex_idx)
                break;
            vertex_idx_lst = vertex_idx;

            Eigen::MatrixXd vertex_coord(8, 3);
            for (int i = 0; i < 8; i++){
                int idx_x = std::max(std::min(vertex_idx(i, 0), PlannerParameter::_max_x_id - 1), 0);
                int idx_y = std::max(std::min(vertex_idx(i, 1), PlannerParameter::_max_y_id - 1), 0);
                int idx_z = std::max(std::min(vertex_idx(i, 2), PlannerParameter::_max_z_id - 1), 0);

                Eigen::Vector3i idx(idx_x, idx_y, idx_z);
                Eigen::Vector3d pos = collision_map_->GridIndexToLocation(idx);
                vertex_coord.row(i) = pos;
            }

            cubeMax_ = setVertex(vertex_coord, PlannerParameter::_resolution);

            if(isContain(_lstcube, cubeMax_))
                return std::make_pair(_lstcube, false);
            iter++;
        }
        return std::make_pair(cubeMax_, true);
    }

    void RegionSegmentation::segmentation(const std::vector<NodePtr> &_path_node_pool){
        segmentation_res.clear();
        Eigen::Vector3d path_coord;

        cubePtr lstcube = new Cube();
        for (int i = 0; i < _path_node_pool.size(); i++){
            path_coord = _path_node_pool[i]->position;
            cubePtr tmp_cube = cubeGeneration(path_coord);
            auto inf_cube = cubeInflate(tmp_cube, lstcube);
            if(inf_cube.second == false)
                continue;
            tmp_cube = inf_cube.first;
            segmentation_res.push_back(tmp_cube);
            lstcube = tmp_cube;
        }
        std::cout << "segmentation res = " << segmentation_res.size() << std::endl;
        segmentationVis(segmentation_res);
        // return segmentation_res;
    }

    void RegionSegmentation::segmentationVis(const std::vector<cubePtr>& _segmentation_res){
        int id = 0;
        segmentation_voxel_.markers.clear();
        for(int i = 0; i < _segmentation_res.size(); i++){
            double pos_x = (_segmentation_res[i]->vertex_(0, 0) + _segmentation_res[i]->vertex_(3, 0)) / 2;
            double pos_y = (_segmentation_res[i]->vertex_(0, 1) + _segmentation_res[i]->vertex_(1, 1)) / 2;
            double pos_z = (_segmentation_res[i]->vertex_(0, 2) + _segmentation_res[i]->vertex_(4, 2)) / 2;
            double scale_x = std::abs(_segmentation_res[i]->vertex_(0, 0) - _segmentation_res[i]->vertex_(3, 0));
            double scale_y = std::abs(_segmentation_res[i]->vertex_(0, 1) - _segmentation_res[i]->vertex_(1, 1));
            double scale_z = std::abs(_segmentation_res[i]->vertex_(0, 2) - _segmentation_res[i]->vertex_(4, 2));
            voxel_ = cubeVis(pos_x, pos_y, pos_z, scale_x, scale_y, scale_z, id);
            segmentation_voxel_.markers.push_back(voxel_);
            id++;
        }
        segmentation_res_vis_.publish(segmentation_voxel_);
    }

    visualization_msgs::Marker RegionSegmentation::cubeVis(const double x, const double y, const double z, const double scale_x,
                                               const double scale_y, const double scale_z, const int id){
        voxel_.header.frame_id = "world";
        voxel_.header.stamp = ros::Time::now();
        voxel_.ns = "map";
        voxel_.id = id;
        voxel_.type = visualization_msgs::Marker::CUBE;
        voxel_.action = visualization_msgs::Marker::ADD;

        voxel_.pose.position.x = x;
        voxel_.pose.position.y = y;
        voxel_.pose.position.z = z;
        voxel_.pose.orientation.x = 0.0;
        voxel_.pose.orientation.y = 0.0;
        voxel_.pose.orientation.z = 0.0;
        voxel_.pose.orientation.w = 1.0;

        voxel_.scale.x = scale_x;
        voxel_.scale.y = scale_y;
        voxel_.scale.z = scale_z;

        voxel_.color.a = 0.2;
        voxel_.color.r = 0.5;
        voxel_.color.g = 1.0;
        voxel_.color.b = 0.5;
        return voxel_;
    }
}