#ifndef BASIC_A_H
#define BASIC_A_H

#include <iostream>
#include <vector>
#include <unordered_map>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Eigen>
#include <queue>

#include "collision_model.h"
#include "bazier_traj/planner_parameter.h"

#define inf 1>>30

namespace planner{

#define NOT_EXPAND 'a'
#define IN_OPEN_SET 'b'
#define IN_CLOSE_SET 'c'

struct Node{
    Node* parent;
    // Eigen::VectorXd node_space;
    Eigen::Vector3d position;
    double theta{};
    double radius{};
    char node_state;
    double g_score{};
    double f_score{};
    double h_score{};
    double occupancy{};
    Eigen::Vector3i index;

    Node(Eigen::Vector3i _index, Eigen::Vector3d _pos){
        index = _index;
        position = _pos;
        g_score = inf;
        f_score = inf;
        h_score = inf;
        parent = NULL;
        node_state = NOT_EXPAND;
    }
    Node()
    {
        parent = NULL;
        node_state = NOT_EXPAND;
    }
    // Node(){}
    ~Node(){}
};
typedef Node* NodePtr;


class NodeComparator{
public:
    bool operator()(NodePtr& node1, NodePtr& node2){
        return node1 -> h_score > node2 -> h_score;
    }
};

template <typename T>
struct matrix_hash0 : public std::unary_function<T,size_t>{
    size_t operator()(T const& matrix) const{
        size_t seed = 0;
        for(size_t i=0; i<matrix.size(); ++i){
            auto elem = *(matrix.data() + i);
            seed ^= std::hash<typename T::Scalar>() (elem) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
        }
        return seed;
    }
};

class NodeHashTable{
private:
    std::unordered_map<Eigen::Vector3i, NodePtr, matrix_hash0<Eigen::Vector3i>> data_3d_;
    std::unordered_map<Eigen::VectorXd, NodePtr, matrix_hash0<Eigen::Vector4i>> data_4d_;

public:
    NodeHashTable(){};
    ~NodeHashTable(){};

    void insert(Eigen::Vector3i idx, NodePtr node){
        data_3d_.insert(std::make_pair(idx,node));
    }
    // void insert(Eigen::VectorXd idx, int time_idx, NodePtr node){
    //     data_4d_.insert(std::make_pair((Eigen::Vector4i(idx(0),idx(1),idx(2), time_idx)), node));
    // }
    NodePtr find(Eigen::Vector3i idx){
        Eigen::Vector3d temp;
        auto iter = data_3d_.find(idx);
        return iter == data_3d_.end() ? NULL : iter -> second;
    }
    // NodePtr find(Eigen::Vector3i idx, int time_idx){
    //     auto iter = data_4d_.find(Eigen::Vector4i(idx(0),idx(1),idx(2),time_idx));
    //     return iter == data_4d_.end() ? NULL : iter -> second;
    // }
    void clear() {
        data_3d_.clear();
        data_4d_.clear();
    }
};

class BasedAstar{
public:
    //tag: search gScore fScore 三个需要调整node_space 和 position
    BasedAstar() = default;
    virtual int search(NodePtr _end_pt) = 0;
    virtual void retrievePath(NodePtr cur_node) = 0;
    virtual void setParam() = 0;
    virtual double gScoreCalc(NodePtr _cur_node, NodePtr _pro_node) = 0;
    virtual double fScoreCalc(NodePtr _cur_node) = 0;
    virtual void resetPath() = 0;
    // virtual void init() = 0;
    // virtual bool collisionChecking() = 0;

protected:
    NodePtr start_pt_, end_pt_;
    CollisionPtr robot_model_ = std::make_shared<CollisionModel>();
    std::vector<NodePtr> path_node_pool_;
    std::priority_queue<NodePtr, std::vector<NodePtr>, NodeComparator> open_set_;
    NodeHashTable expanded_nodes_;
};

}

#endif