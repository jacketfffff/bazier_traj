#include "bazier_traj/astar.h"


namespace planner{

    AStar::AStar(){

    }

    AStar::~AStar(){

    }

    std::vector<NodePtr> AStar::getPath(){
        return path_node_pool_;
    }

    void AStar::setParam(){
        resolution_ = PlannerParameter::_resolution;
        inv_resolution_ = PlannerParameter::_inv_resolution;
        map_size_ << PlannerParameter::_x_local_size, PlannerParameter::_y_local_size, PlannerParameter::_z_local_size;
        map_idx_ << ceil(map_size_(0) * inv_resolution_),
                    ceil(map_size_(1) * inv_resolution_),
                    ceil(map_size_(2) * inv_resolution_);
    }

    void AStar::initLocalMap(){
        int x_size = map_idx_(0);
        int y_size = map_idx_(1);
        int z_size = map_idx_(2);
        GridNodeMap = new NodePtr **[x_size];
        for (int i = 0; i < x_size; i++){
             GridNodeMap[i] = new NodePtr *[y_size];
            for (int j = 0; j < y_size; j++){
                GridNodeMap[i][j] = new NodePtr[z_size];
                for (int k = 0; k < z_size; k++){
                    Eigen::Vector3i tmpIdx(i, j, k);
                    Eigen::Vector3d pos = index2coord(tmpIdx);
                    GridNodeMap[i][j][k] = new Node(tmpIdx, pos);
                }
            }
        }
    }

    void AStar::importLocalMap(sdf_tools::CollisionMapGrid *_local_map){
        Eigen::Vector3d coord;
        for (int64_t i = 0; i < map_idx_(0); ++i){
            for (int64_t j = 0; j < map_idx_(1); j++){
                for (int64_t k = 0; k < map_idx_(2); k++){
                    coord(0) = static_cast<double>((i + 0.5) * resolution_) - map_size_(0) / 2.0;
                    coord(1) = static_cast<double>((j + 0.5) * resolution_) - map_size_(1) / 2.0;
                    coord(2) = static_cast<double>((k + 0.5) * resolution_) - map_size_(2);
                    // std::cout << coord(0) << " , " << coord(1) << " , " << coord(2) << " , " << std::endl;
                    Eigen::Vector3i index = coord2index(coord);
                    if (index(0) >= map_idx_(0) || index(1) >= map_idx_(1) || index(2) >= map_idx_(2) || index(0) < 0 || index(1) < 0 || index(2) < 0)
                    {
                        std::cout << " Index Error, Check..." << std::endl;
                        continue;
                    }
                    NodePtr ptr = GridNodeMap[index(0)][index(1)][index(2)];
                    ptr->occupancy = _local_map->Get(i, j, k).first.occupancy;
                    // std::cout << ptr -> occupancy << std::endl;

                }
            }
        }
        mapVis(GridNodeMap);
    }

    visualization_msgs::Marker AStar::pathVoxelVis(const double x, const double y, const double z, const int id){
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

        voxel_.scale.x = PlannerParameter::_resolution;
        voxel_.scale.y = PlannerParameter::_resolution;
        voxel_.scale.z = PlannerParameter::_resolution;

        voxel_.color.a = 1;
        voxel_.color.r = 0.5;
        voxel_.color.g = 1.0;
        voxel_.color.b = 0.5;
        return voxel_;
    }

    void AStar::pathVis(const std::vector<NodePtr> &_path){
        int id = 0;
        path_.markers.clear();
        for (int i = 0; i < _path.size(); i++){
            double x = _path[i]->position(0);
            double y = _path[i]->position(1);
            double z = _path[i]->position(2);
            path_voxel_ = pathVoxelVis(x, y, z, id);
            path_.markers.push_back(path_voxel_);
            ++id;
        }
        path_vis_.publish(path_);
        // std::cout << "test !!! " << std::endl;
    }

    void AStar::mapVis(NodePtr*** _map){
        int id = 0;
        voxel_map_.markers.clear();
        for (int i = 0; i < map_idx_(0); ++i){
            for (int j = 0; j < map_idx_(1); ++j){
                for (int k = 0; k < map_idx_(2); ++k){
                    // std::cout << _map[i][j][k]->occupancy << std::endl;
                    if (_map[i][j][k]->occupancy >= 0.5)
                    {
                        // std::cout << " Publish Test Map " << std::endl;
                        visualization_msgs::Marker box = voxelVis(i, j, k, id);
                        voxel_map_.markers.push_back(box);
                        ++id;
                    }
                }
            }
        }
        local_map_vis_.publish(voxel_map_);
    }

    visualization_msgs::Marker AStar::voxelVis(int x, int y, int z, int id){
        voxel_.header.frame_id = "world";
        voxel_.header.stamp = ros::Time::now();
        voxel_.ns = "map";
        voxel_.id = id;
        voxel_.type = visualization_msgs::Marker::CUBE;
        voxel_.action = visualization_msgs::Marker::ADD;

        voxel_.pose.position.x = (x + 0.5) * PlannerParameter::_resolution - PlannerParameter::_x_local_size / 2.0 + 0.25;
        voxel_.pose.position.y = (y + 0.5) * PlannerParameter::_resolution - PlannerParameter::_y_local_size / 2.0 + 0.25;
        voxel_.pose.position.z = (z + 0.5) * PlannerParameter::_resolution - PlannerParameter::_z_local_size / 2.0 + 0.5;
        voxel_.pose.orientation.x = 0.0;
        voxel_.pose.orientation.y = 0.0;
        voxel_.pose.orientation.z = 0.0;
        voxel_.pose.orientation.w = 1.0;

        voxel_.scale.x = PlannerParameter::_resolution;
        voxel_.scale.y = PlannerParameter::_resolution;
        voxel_.scale.z = PlannerParameter::_resolution;

        voxel_.color.a = 0.8;
        voxel_.color.r = 0;
        voxel_.color.g = 1.0;
        voxel_.color.b = 0;
        return voxel_;
    }

    void AStar::mapTest(ros::NodeHandle &_nh){
        local_map_vis_ = _nh.advertise<visualization_msgs::MarkerArray>("test_map", 10);
        path_vis_ = _nh.advertise<visualization_msgs::MarkerArray>("path_test", 10);
    }

    void AStar::resetLocalMap(){
        while(!open_set_.empty()){
            open_set_.pop();
        }
    }

    double AStar::gScoreCalc(NodePtr _cur_node, NodePtr _pro_node){
        double dx = pow(_pro_node->position[0] - _cur_node->position[0], 2);
        double dy = pow(_pro_node->position[1] - _cur_node->position[1], 2);
        double dz = pow(_pro_node->position[2] - _cur_node->position[2], 2);
        double tmp_g_score = sqrt(dx + dy + dz);
        return tmp_g_score + _pro_node->g_score;
    }

    double AStar::fScoreCalc(NodePtr _cur_node){
        double dx = pow(end_pt_->position(0) - _cur_node->position(0), 2);
        double dy = pow(end_pt_->position(1) - _cur_node->position(1), 2);
        double dz = pow(end_pt_->position(2) - _cur_node->position(2), 2);
        double tmp_heu_dist = sqrt(dx + dy + dz);
        return tmp_heu_dist;
    }

    Eigen::Vector3i AStar::coord2index(const Eigen::Vector3d &coord){
        // todo: 确认z方向要不要除以2.0
        Eigen::Vector3i idx;
        idx << std::min(std::max(int((coord(0) + map_size_(0) / 2) * inv_resolution_), 0), map_idx_(0) - 1),
               std::min(std::max(int((coord(1) + map_size_(1) / 2) * inv_resolution_), 0), map_idx_(1) - 1),
               std::min(std::max(int((coord(2) + map_size_(2)) * inv_resolution_), 0), map_idx_(2) - 1);
        return idx;
    }

    Eigen::Vector3d AStar::index2coord(const Eigen::Vector3i &index){
        Eigen::Vector3d coord;
        coord(0) = ((double)index(0) + 0.5) * resolution_ - map_size_(0) / 2.0;
        coord(1) = ((double)index(1) + 0.5) * resolution_ - map_size_(1) / 2.0;
        coord(2) = ((double)index(2) + 0.5) * resolution_ - map_size_(2);
        return coord;
    }

    bool AStar::collisionChecking(NodePtr ***_node_map, const Eigen::Vector3i pos_idx, const char symbol){
        Eigen::Vector3i model_size;
        if (symbol == CUBE_)
            model_size = robot_model_ -> setModel(CUBE_);
        else if (symbol == CYLINDER_)
            model_size = robot_model_ -> setModel(CYLINDER_);
        std::cout << "test!!!!!!" << std::endl;
        for (int i = pos_idx(0) - model_size(0); i <= pos_idx(0) + model_size(0); ++i)
            for (int j = pos_idx(1) - model_size(1); j <= pos_idx(1) + model_size(1); ++j)
                for (int k = pos_idx(2) - model_size(2); k <= pos_idx(2); ++k){
                    if(i > map_idx_(0)-1 || j > map_idx_(1) - 1 || k > map_idx_(2) - 1 ||
                       i < 0 || j < 0 || k < 0)
                        return true;
                    if (_node_map[i][j][k]->occupancy > 0.5)
                        return false;
        }
        return true;
    }

    void AStar::resetPath(){
        path_node_pool_.clear();
    }

    void AStar::retrievePath(NodePtr _end_pt){
        NodePtr tmp_node = _end_pt;
        path_node_pool_.push_back(tmp_node);
        while(tmp_node->parent != NULL){
            tmp_node = tmp_node->parent;
            // std::cout <<"Path pos = " << tmp_node->position(0) << " " << tmp_node->position(1) << " " << tmp_node->position(2) << std::endl;
            path_node_pool_.push_back(tmp_node);
        }
        reverse(path_node_pool_.begin(), path_node_pool_.end());
        pathVis(path_node_pool_);
    }

    NodePtr AStar::setTarget(Eigen::Vector3d _end_pos){
        Eigen::Vector3i _end_idx = coord2index(_end_pos);
        end_pt_ = new Node(_end_idx, _end_pos);
        return end_pt_;
    }

    void AStar::clearNodeState(){
        int x_size = map_idx_(0);
        int y_size = map_idx_(1);
        int z_size = map_idx_(2);
        for (int i = 0; i < x_size; i++){
            for (int j = 0; j < y_size; j++){
                for (int k = 0; k < z_size; k++){
                    GridNodeMap[i][j][k] -> node_state = NOT_EXPAND;
                    GridNodeMap[i][j][k]->g_score = inf;
                    GridNodeMap[i][j][k]->f_score = inf;
                    GridNodeMap[i][j][k]->h_score = inf;
                    GridNodeMap[i][j][k]->parent = NULL;
                }
            }
        }
    }

    int AStar::search(NodePtr _end_pt){
        // ros::Time time_1 = ros::Time::now();
        // init start_pt
        clearNodeState();
        NodePtr cur_node = NULL;
        start_pt_ = new Node();
        start_pt_->position << 0.0, 0.0, 0.0;
        start_pt_->index = coord2index(start_pt_->position);
        std::cout << "start pos " << start_pt_->index(0) << " " << start_pt_->index(1) << " " << start_pt_->index(2) << std::endl;
        start_pt_->g_score = 0;
        start_pt_->f_score = fScoreCalc(start_pt_);
        start_pt_->h_score = start_pt_->g_score + start_pt_->f_score;
        start_pt_->parent = NULL;
        // // tag: 起始点由里程计从tf中读取, 相对于局部地图系的坐标
        // start_pt_->index = coord2index(start_pt_->position);
        open_set_ = {};
        open_set_.push(start_pt_);
        start_pt_->node_state = IN_OPEN_SET;
        while (!open_set_.empty())
        {
            cur_node = open_set_.top();
            std::cout << "cur node idx = " << cur_node->index(0) << " " << cur_node->index(1) << " " << cur_node->index(2) << std::endl;
            // std::cout << cur_node->position(0) << " " << cur_node->position(1) << " " << cur_node->position(2) << std::endl;
            cur_node->node_state = IN_CLOSE_SET;
            open_set_.pop();
            if(cur_node -> index == _end_pt -> index){
                ROS_INFO("Reach Goal");
                resetPath();
                retrievePath(cur_node);
                return true;
            }
            for (int dx = -1; dx < 2; dx++){
                for (int dy = -1; dy < 2; dy++){
                    // for (int dz = -1; dz < 2; dz++){
                        // if(dx == 0 && dy == 0 && dz == 0)
                        if(dx == 0 && dy == 0)
                            continue;
                       // std::cout << " test !!! " << std::endl;
                        Eigen::Vector3i neighbor_idx;
                        // std::cout << "cur node idx = " << cur_node->index(0) << " " << cur_node->index(1) << " " << cur_node->index(2) << std::endl;
                            neighbor_idx << cur_node->index(0) + dx,
                                cur_node->index(1) + dy,
                                cur_node->index(2);
                            if (neighbor_idx(0) >= map_idx_(0) || neighbor_idx(1) >= map_idx_(1) || neighbor_idx(2) >= map_idx_(2) ||
                                neighbor_idx(0) < 0 || neighbor_idx(1) < 0 || neighbor_idx(2) < 0)
                                continue;
                            NodePtr neighbor_node = GridNodeMap[neighbor_idx(0)][neighbor_idx(1)][neighbor_idx(2)];
                            // if (neighbor_node->occupancy > 0.5)
                                // continue;
                            //                     //tag: collision checking
                            bool flag = collisionChecking(GridNodeMap, neighbor_idx, PlannerParameter::_robot_model);
                            if (!flag)
                                continue;
                            if (neighbor_node->node_state == NOT_EXPAND)
                            {
                                neighbor_node->position = index2coord(neighbor_idx);
                                // std::cout << "neighbot idx = " << neighbor_idx(0) << " " << neighbor_idx(1) << " " << neighbor_idx(2) << std::endl;
                                // std::cout << "neighbor node pos = " << neighbor_node->position(0) << " " << neighbor_node->position(1) << " " << neighbor_node->position(2) << std::endl;

                                neighbor_node->g_score = gScoreCalc(cur_node, neighbor_node);
                                neighbor_node->f_score = fScoreCalc(neighbor_node);
                                neighbor_node->h_score = neighbor_node->g_score + neighbor_node->f_score;
                                neighbor_node->parent = cur_node;
                                neighbor_node->node_state = IN_OPEN_SET;
                                open_set_.push(neighbor_node);
                        }
                        else if(neighbor_node -> node_state == IN_OPEN_SET){
                            double temp_g = gScoreCalc(cur_node, neighbor_node);
                            if(temp_g < neighbor_node -> g_score){
                                neighbor_node->g_score = temp_g;
                                neighbor_node->f_score = fScoreCalc(neighbor_node);
                                neighbor_node->h_score = neighbor_node->g_score + neighbor_node->f_score;
                                neighbor_node->parent = cur_node;
                            }
                        }
                        else
                            continue;


                // }
                }
            }
        }
        ROS_INFO("No Path");
        return 0;
    }


}
