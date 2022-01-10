#ifndef BEZIER_TRAJECTORY_H
#define BEZIER_TRAJECTORY_H

#include <ros/console.h>
#include <nav_msgs/Path.h>
#include <Eigen/Dense>
#include <vector>
#include "bazier_traj/planner_parameter.h"
#include "mosek.h"
#include "region_segmentation.h"
//bezier trajectory generator
namespace planner{

    class Bernstein{
        private:
            std::vector<Eigen::MatrixXd> MQMList_, MList_;
            std::vector<Eigen::MatrixXd> FMList_;
            std::vector<Eigen::VectorXd> CList_, CvList_, CaList_, CjList_;

            int order_min_, order_max_;
            double min_order_;

        public:
            Bernstein();

            Bernstein(int poly_order_min, int poly_order_max, double min_order);

            ~Bernstein();

            int setParam(int poly_order_min, int poly_order_max, double min_order);

            Eigen::MatrixXd CholeskyDecomp(Eigen::MatrixXd Q);

            inline std::vector<Eigen::MatrixXd> getM() { return MList_; }
            inline std::vector<Eigen::MatrixXd> getMQM() { return MQMList_; }
            inline std::vector<Eigen::MatrixXd> getFM() { return FMList_; }
            inline std::vector<Eigen::VectorXd> getC() { return CList_; }
            inline std::vector<Eigen::VectorXd> getC_v() { return CvList_; }
            inline std::vector<Eigen::VectorXd> getC_a() { return CaList_; }
            inline std::vector<Eigen::VectorXd> getC_j() { return CjList_; }
    };

    class TrajectoryGenerator{
        public:
            TrajectoryGenerator(){}
            ~TrajectoryGenerator(){}

            // Use Bezier curve for the trajectory
            int BezierPolyCoeffGeneration(
                const std::vector<cubePtr> &_region,
                const Eigen::MatrixXd &_MQM,
                const Eigen::MatrixXd &_pos,
                const Eigen::MatrixXd &_vel,
                const Eigen::MatrixXd &_acc,
                const double _maxVel,
                const double _maxAcc,
                const int _traj_order,
                const double _minimize_order,
                const double _margin,
                double &_obj,
                Eigen::MatrixXd &_PolyCoeff);
    };
}

#endif // BEZIER_TRAJECTORY_H