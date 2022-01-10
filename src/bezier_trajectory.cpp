#include "bezier_trajectory.h"

namespace planner{
    Bernstein::Bernstein(){

    }

    Bernstein::Bernstein(int _poly_order_min, int _poly_order_max, double _min_order){
        order_min_ = _poly_order_min;
        order_max_ = _poly_order_max;
        min_order_ = _min_order;
    }

    Bernstein::~Bernstein(){

    }
    //?: 矩阵分解方法存疑
    Eigen::MatrixXd Bernstein::CholeskyDecomp(Eigen::MatrixXd _Q){
        Eigen::MatrixXd F, Ft;
        Eigen::LDLT<Eigen::MatrixXd> ldlt(_Q);
        F = ldlt.matrixL();
        F = ldlt.transpositionsP().transpose() * F;
        F *= ldlt.vectorD().array().sqrt().matrix().asDiagonal();
        Ft = F.transpose();
        return Ft;
    }

    int Bernstein::setParam(int _poly_order_min, int _poly_order_max, double _min_order){
        int ret = (_poly_order_min >= 3 && _poly_order_max <= 13) ? 1 : -1;

        order_min_ = _poly_order_min;
        order_max_ = _poly_order_max;
        min_order_ = _min_order;

        const static auto factorial = [](int n){
            int fact = 1;
            for (int i = n; i > 0; i--)
                fact *= i;
            return fact;
        };
        const static auto combinatorial = [](int n, int k){
            return factorial(n) / (factorial(k) * factorial(n - k));
        };

        CList_.clear();
        CvList_.clear();
        CaList_.clear();
        CjList_.clear();

        for (int order = 0; order <= order_max_; order++){
            Eigen::MatrixXd M;
            Eigen::MatrixXd Q, Q_l, Q_u;
            Eigen::MatrixXd MQM;

            Eigen::VectorXd C;
            Eigen::VectorXd C_v;
            Eigen::VectorXd C_a;
            Eigen::VectorXd C_j;
            //计算M矩阵，根据多项式阶数不同产生固定映射
            switch(order){
                case 0:{
                    M << 1;
                    break;
                }
                case 1:{
                    M << -1,  0,
                         -1,  1;
                    break;
                }
                case 2:{
                    M << -1,  0,  0,
                         -2,  2,  0,
                          1, -2,  1;
                    break;
                }
                case 3:{
                    M << -1,  0,  0,  0,
                         -3,  3,  0,  0,
                          3, -6,  3,  0,
                         -1,  3, -3,  1;
                    break;
                }
                case 4:{
                    M <<  1,   0,   0,   0,  0,
                         -4,   4,   0,   0,  0,
                          6, -12,   6,   0,  0,
                         -4,  12, -12,   4,  0,
                          1,  -4,   6,  -4,  1;
                    break;
                }
                case 5:{
                    M << 1,   0,   0,   0,  0,  0,
                        -5,   5,   0,   0,  0,  0,
                        10, -20,  10,   0,  0,  0,
                       -10,  30, -30,  10,  0,  0,
                         5, -20,  30, -20,  5,  0,
                        -1,   5, -10,  10, -5,  1;
                    break;
                }
                case 6:{
                    M << 1,   0,   0,   0,   0,  0,  0,
                        -6,   6,   0,   0,   0,  0,  0,
                        15, -30,  15,   0,   0,  0,  0,
                       -20,  60, -60,  20,   0,  0,  0,
                        15, -60,  90, -60,  15,  0,  0,
                        -6,  30, -60,  60, -30,  6,  0,
                         1,  -6,  15, -20,  15, -6,  1;
                    break;
                }
                case 7:{
                    M << 1,    0,    0,    0,    0,   0,   0,   0,
                        -7,    7,    0,    0,    0,   0,   0,   0,
                        21,   42,   21,    0,    0,   0,   0,   0,
                       -35,  105, -105,   35,    0,   0,   0,   0,
                        35, -140,  210, -140,   35,   0,   0,   0,
                       -21,  105, -210,  210, -105,  21,   0,   0,
                         7,  -42,  105, -140,  105, -42,   7,   0,
                        -1,    7,  -21,   35,  -35,  21,  -7,   1;
                    break;
                }
                case 8:{
                    M << 1,    0,    0,    0,    0,    0,   0,   0,   0,
                        -8,    8,    0,    0,    0,    0,   0,   0,   0,
                        28,  -56,   28,    0,    0,    0,   0,   0,   0,
                       -56,  168, -168,   56,    0,    0,   0,   0,   0,
                        70, -280,  420, -280,   70,    0,   0,   0,   0,
                       -56,  280, -560,  560, -280,   56,   0,   0,   0,
                        28, -168,  420, -560,  420, -168,  28,   0,   0,
                        -8,   56, -168,  280, -280,  168, -56,   8,   0,
                         1,   -8,   28,  -56,   70,  -56,  28,  -8,   1;
                    break;
                }
                case 9:{
                    M << 1,    0,     0,     0,     0,    0,    0,     0,     0,    0,
                        -9,    9,     0,     0,     0,    0,    0,     0,     0,    0,
                        36,  -72,    36,     0,     0,    0,    0,     0,     0,    0,
                       -84,  252,  -252,    84,     0,    0,    0,     0,     0,    0,
                       126, -504,   756,  -504,   126,    0,    0,     0,     0,    0,
                      -126,  630, -1260,  1260,  -630,  126,    0,     0,     0,    0,
                        84, -504,  1260, -1680,  1260, -504,   84,     0,     0,    0,
                       -36,  252,  -756,  1260, -1260,  756, -252,    36,     0,    0,
                         9,  -72,   252,  -504,   630, -504,  252,   -72,     9,    0,
                        -1,    9,   -36,    84,  -126,  126,  -84,    36,    -9,    1;
                    break;
                }
                case 10:{
                    M <<  1,     0,      0,     0,      0,     0,    0,     0,     0,    0,   0,
                        -10,    10,      0,     0,      0,     0,    0,     0,     0,    0,   0,
                         45,   -90,     45,     0,      0,     0,    0,     0,     0,    0,   0,
                       -120,   360,   -360,   120,      0,     0,    0,     0,     0,    0,   0,
                        210,  -840,   1260,  -840,    210,     0,    0,     0,     0,    0,   0,
                       -252,  1260,  -2520,  2520,  -1260,   252,    0,     0,     0,    0,   0,
                        210, -1260,   3150, -4200,   3150, -1260,  210,     0,     0,    0,   0,
                       -120,   840,  -2520,  4200,  -4200,  2520, -840,   120,     0,    0,   0,
                         45,  -360,   1260, -2520,   3150, -2520, 1260,  -360,    45,    0,   0,
                        -10,    90,   -360,   840,  -1260,  1260, -840,   360,   -90,   10,   0,
                          1,   -10,     45,  -120,    210,  -252,  210,  -120,    45,  -10,   1;
                    break;
                }
        }

            int poly_num_1D = order + 1;
            M.resize(poly_num_1D, poly_num_1D);
            Q_l = Eigen::MatrixXd::Zero(poly_num_1D, poly_num_1D);
            Q_u = Eigen::MatrixXd::Zero(poly_num_1D, poly_num_1D);
            Q   = Eigen::MatrixXd::Zero(poly_num_1D, poly_num_1D);
            C.resize(poly_num_1D);
            C_v.resize(poly_num_1D - 1);

            if(order > 1)
                C_a.resize(poly_num_1D - 2);
            if(order > 2)
                C_j.resize(poly_num_1D - 3);
            int min_order_l = std::floor(_min_order);
            int min_order_u = std::ceil(min_order_);

            if(poly_num_1D > min_order_l){
                for (int i = min_order_l; i < poly_num_1D; i++){
                    for (int j = min_order_l; j < poly_num_1D; j++){
                        int coeff = 1;
                        int d = min_order_l - 1;
                        while(d >= 0){
                            coeff *= (i - d) * (j - d);
                            d--;
                        }
                        Q_l(i, j) = double(coeff);
                    }
                }
            }
            if(poly_num_1D > min_order_u){
                for (int i = min_order_u; i < poly_num_1D; i++){
                    for (int j = min_order_u; j < poly_num_1D; j++){
                        int coeff = 1;
                        int d = min_order_u - 1;
                        while(d >= 0){
                            coeff *= (i - d) * (j - d);
                            d--;
                        }
                        Q_u(i, j) = double(coeff);
                    }
                }
            }
            if(min_order_l == min_order_u)
                Q = Q_u;
            else
                Q = (min_order_ - min_order_l) * Q_u + (min_order_u - min_order_) * Q_l;

            MQM = M.transpose() * Q * M;
            Eigen::MatrixXd F = CholeskyDecomp(Q);
            Eigen::MatrixXd FM = F * M;

            int n = order;
            for (int k = 0; k <= n; k++){
                C(k) = combinatorial(n, k);
                if(k <= n - 1)
                    C_v(k) = combinatorial(n - 1, k);
                if(k <= n - 2)
                    C_a(k) = combinatorial(n - 2, k);
                if(k <= n - 3)
                    C_j(k) = combinatorial(n - 3, k);
            }
            MList_.push_back(M);
            MQMList_.push_back(MQM);
            FMList_.push_back(FM);

            CList_.push_back(C);
            CvList_.push_back(C_v);
            CaList_.push_back(C_a);
            CjList_.push_back(C_j);
        }
        return ret;
    }

    int TrajectoryGenerator::BezierPolyCoeffGeneration(
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
        Eigen::MatrixXd &_PolyCoeff){

        double initScale = _region.front()->t_;
        double lstScale = _region.back()->t_;
        int segment_num = _region.size();
        //n阶贝塞尔曲线，要n+1个控制点，三维上有3*（n+1）坐标，二维上有2（n+1）个
        int n_ploy = _traj_order + 1;
        int s1d1CtrlPt_num = n_ploy;
        // int s1CtrlPt_num = 3 * s1d1CtrlPt_num;
        int s1CtrlPt_num = 2 * s1d1CtrlPt_num;
        //等式约束： 包括位置约束和连续性约束， jerk作为目标函数，pva都有约束项，轨迹维度是二维
        // int equ_con_s_num = 3 * 3;
        int equ_con_s_num = 2 * 3;
        // int equ_con_e_num = 3 * 3;
        int equ_con_e_num = 2 * 3;
        // int equ_con_continuity_num = 3 * 3 * (segment_num - 1);
        int equ_con_continuity_num = 2 * 3 * (segment_num - 1);
        int equ_con_num = equ_con_s_num + equ_con_e_num + equ_con_continuity_num;
        //速度和加速度的约束是不等式约束，主要限制最大速度的范围
        // int vel_con_num = 3 * _traj_order * segment_num;
        int vel_con_num = 2 * _traj_order * segment_num;
        int acc_con_num = 2 * (_traj_order - 1) * segment_num;

        int high_order_con_num = vel_con_num + acc_con_num;

        int con_num = equ_con_num + high_order_con_num;
        int ctrlPt_num = segment_num * s1CtrlPt_num;
        /************以上，整体搭建了优化问题各类约束的数量信息***********/
        double x_var[ctrlPt_num];
        double primalobj;

        MSKrescodee r;

        std::vector<std::pair<MSKboundkeye, std::pair<double, double>>> con_bdk;
        for (int i = 0; i < vel_con_num; i++){
            std::pair<MSKboundkeye, std::pair<double, double>> cb_ie = std::make_pair(MSK_BK_RA, std::make_pair(-_maxVel, +_maxVel));
            con_bdk.push_back(cb_ie);
        }
        for (int i = 0; i < acc_con_num; i++){
            std::pair<MSKboundkeye, std::pair<double, double>> cb_ie = std::make_pair(MSK_BK_RA, std::make_pair(-_maxAcc, +_maxAcc));
            con_bdk.push_back(cb_ie);
        }
        for (int i = 0; i < equ_con_num; i++){
            double beq_i;
            if(i < 2)
                beq_i = _pos(0, i);
            else if(i >= 2 && i < 4)
                beq_i = _vel(0, i - 2);
            else if(i >= 4 && i < 6)
                beq_i = _acc(0, i - 4);
            else if(i >= 6 && i < 8)
                beq_i = _pos(0, i - 6);
            else if(i >= 8 && i < 10)
                beq_i = _vel(0, i - 8);
            else if(i >= 10 && i < 12)
                beq_i = _acc(0, i - 10);
            else
                beq_i = 0.0;
            std::pair<MSKboundkeye, std::pair<double, double>> cb_eq = std::make_pair(MSK_BK_FX, std::make_pair(beq_i, beq_i));
            con_bdk.push_back(cb_eq);
        }

        std::vector<std::pair<MSKboundkeye, std::pair<double, double>>> var_bdk;
        for (int k = 0; k < segment_num; k++){
            cubePtr cube_ = _region[k];
            double scale_k = cube_->t_;
            for (int i = 0; i < 2; i++){
                for (int j = 0; j < n_ploy; j++){
                    std::pair<MSKboundkeye, std::pair<double, double>> vb_x;
                    double lo_bound, up_bound;
                    if( i == 0){
                        lo_bound = cube_->vertex_(3, i) / scale_k;
                        up_bound = cube_->vertex_(0, i) / scale_k;
                    }
                    if(i == 1){
                        lo_bound = cube_->vertex_(0, i) / scale_k;
                        up_bound = cube_->vertex_(1, i) / scale_k;
                    }

                    vb_x = std::make_pair(MSK_BK_RA, std::make_pair(lo_bound, up_bound));
                    var_bdk.push_back(vb_x);
                }
            }
        }

        MSKint32t j, i;
        MSKenv_t env;
        MSKtask_t task;

        r = MSK_makeenv(&env, NULL);
        r = MSK_maketask(env, con_num, ctrlPt_num, &task);
        //线程数量， 非凸问题容差，内点法求解容差
        MSK_putintparam(task, MSK_IPAR_NUM_THREADS, 1);
        MSK_putdouparam(task, MSK_DPAR_CHECK_CONVEXITY_REL_TOL, 1e-2);
        MSK_putdouparam(task, MSK_DPAR_INTPNT_TOL_DFEAS, 1e-4);
        MSK_putdouparam(task, MSK_DPAR_INTPNT_TOL_PFEAS, 1e-4);
        MSK_putdouparam(task, MSK_DPAR_INTPNT_TOL_INFEAS, 1e-4);

        if(r == MSK_RES_OK)
            r = MSK_appendcons(task, con_num);
        if(r == MSK_RES_OK)
            r = MSK_appendvars(task, ctrlPt_num);
        for (j = 0; j < ctrlPt_num && r == MSK_RES_OK; j++){
            if(r == MSK_RES_OK)
                r = MSK_putvarbound(task, j, var_bdk[j].first, var_bdk[j].second.first, var_bdk[j].second.first);
        }
        for (i = 0; i < con_num && r == MSK_RES_OK; i++){
            r = MSK_putconbound(task, i, con_bdk[i].first, con_bdk[i].second.first, con_bdk[i].second.second);
        }
        int row_idx = 0;
        for (int k = 0; k < segment_num; k++){
            for (int i = 0; i < 2; i++){
                for(int p = 0; p < _traj_order; p++){
                    int nzi = 2;
                    MSKint32t asub[nzi];
                    double aval[nzi];

                    aval[0] = -1.0 * _traj_order;
                    aval[1] = 1.0 * _traj_order;

                    asub[0] = k * s1CtrlPt_num + i * s1d1CtrlPt_num + p;
                    asub[1] = k * s1CtrlPt_num + i * s1d1CtrlPt_num + p + 1;

                    r = MSK_putarow(task, row_idx, nzi, asub, aval);
                    row_idx++;
                }
            }
        }

        for (int k = 0; k < segment_num; k++){
            for (int i = 0; i < 2; i++){
                for (int p = 0; p < _traj_order - 1; p++){
                    int nzi = 3;
                    MSKint32t asub[nzi];
                    double aval[nzi];

                    aval[0] = 1.0 * _traj_order * (_traj_order - 1) / _region[k]->t_;
                    aval[1] = -2.0 * _traj_order * (_traj_order - 1) / _region[k]->t_;
                    aval[2] = 1.0 * _traj_order * (_traj_order - 1) / _region[k]->t_;
                    asub[0] = k * s1CtrlPt_num + i * s1d1CtrlPt_num + p;
                    asub[1] = k * s1d1CtrlPt_num + i * s1d1CtrlPt_num + p + 1;
                    asub[2] = k * s1d1CtrlPt_num + i * s1d1CtrlPt_num + p + 2;

                    r = MSK_putarow(task, row_idx, nzi, asub, aval);
                    row_idx++;
                }
            }
        }
        //起点
        for (int i = 0; i < 2; i++){
            int nzi = 1;
            MSKint32t asub[nzi];
            double aval[nzi];
            aval[0] = 1.0 * initScale;
            asub[0] = i * s1d1CtrlPt_num;
            r = MSK_putarow(task, row_idx, nzi, asub, aval);
            row_idx++;
        }
        for (int i = 0; i < 2; i++){
            int nzi = 2;
            MSKint32t asub[nzi];
            double aval[nzi];
            aval[0] = -1.0 * _traj_order;
            aval[1] = 1.0 * _traj_order;
            asub[0] = i * s1d1CtrlPt_num;
            asub[1] = i * s1d1CtrlPt_num + 1;
            r = MSK_putarow(task, row_idx, nzi, asub, aval);
            row_idx++;
        }
        for (int i = 0; i < 2; i++){
            int nzi = 3;
            MSKint32t asub[nzi];
            double aval[nzi];
            aval[0] =   1.0 * _traj_order * (_traj_order - 1) / initScale;
            aval[1] = - 2.0 * _traj_order * (_traj_order - 1) / initScale;
            aval[2] =   1.0 * _traj_order * (_traj_order - 1) / initScale;
            asub[0] = i * s1d1CtrlPt_num;
            asub[1] = i * s1d1CtrlPt_num + 1;
            asub[2] = i * s1d1CtrlPt_num + 2;
            r = MSK_putarow(task, row_idx, nzi, asub, aval);
            row_idx ++;
        }
        //终点
        for(int i = 0; i < 2; i++)
        {  // loop for x, y, z
            int nzi = 1;
            MSKint32t asub[nzi];
            double aval[nzi];
            asub[0] = ctrlPt_num - 1 - (2 - i) * s1d1CtrlPt_num;
            aval[0] = 1.0 * lstScale;
            r = MSK_putarow(task, row_idx, nzi, asub, aval);
            row_idx ++;
        }
        for(int i = 0; i < 2; i++)
        {
            int nzi = 2;
            MSKint32t asub[nzi];
            double aval[nzi];
            asub[0] = ctrlPt_num - 1 - (2 - i) * s1d1CtrlPt_num - 1;
            asub[1] = ctrlPt_num - 1 - (2 - i) * s1d1CtrlPt_num;
            aval[0] = - 1.0;
            aval[1] =   1.0;
            r = MSK_putarow(task, row_idx, nzi, asub, aval);
            row_idx ++;
        }
        for(int i = 0; i < 3; i++)
        {
            int nzi = 3;
            MSKint32t asub[nzi];
            double aval[nzi];
            asub[0] = ctrlPt_num - 1 - (2 - i) * s1d1CtrlPt_num - 2;
            asub[1] = ctrlPt_num - 1 - (2 - i) * s1d1CtrlPt_num - 1;
            asub[2] = ctrlPt_num - 1 - (2 - i) * s1d1CtrlPt_num;
            aval[0] =   1.0 / lstScale;
            aval[1] = - 2.0 / lstScale;
            aval[2] =   1.0 / lstScale;
            r = MSK_putarow(task, row_idx, nzi, asub, aval);
            row_idx ++;
        }

        //joint points
        int sub_shift = 0;
        double val0, val1;
        for (int k = 0; k < segment_num - 1; k++){
            double scale_k = _region[k]->t_;
            double scale_n = _region[k + 1]->t_;

            val0 = scale_k;
            val1 = scale_n;
            for (int i = 0; i < 3; i++){
                int nzi = 2;
                MSKint32t asub[nzi];
                double aval[nzi];
                // This segment's last control point
                aval[0] = 1.0 * val0;
                asub[0] = sub_shift + (i + 1) * s1d1CtrlPt_num - 1;
                // Next segment's first control point
                aval[1] = -1.0 * val1;
                asub[1] = sub_shift + (i + 1) * s1d1CtrlPt_num;
                r = MSK_putarow(task, row_idx, nzi, asub, aval);
                row_idx++;
            }

            for(int i = 0; i < 2; i++)
            {
                int nzi = 4;
                MSKint32t asub[nzi];
                double aval[nzi];

                // This segment's last velocity control point
                aval[0] = -1.0;
                aval[1] =  1.0;
                asub[0] = sub_shift + (i + 1) * s1d1CtrlPt_num - 2;
                asub[1] = sub_shift + (i + 1) * s1d1CtrlPt_num - 1;
                // Next segment's first velocity control point
                aval[2] =  1.0;
                aval[3] = -1.0;

                asub[2] = sub_shift + (i + 1) * s1CtrlPt_num;
                asub[3] = sub_shift + (i + 1) * s1CtrlPt_num;

                r = MSK_putarow(task, row_idx, nzi, asub, aval);
                row_idx ++;
            }
            val0 = 1.0 / scale_k;
            val1 = 1.0 / scale_n;
            for(int i = 0; i < 2; i++)
            {
                int nzi = 6;
                MSKint32t asub[nzi];
                double aval[nzi];

                // This segment's last velocity control point
                aval[0] =  1.0  * val0;
                aval[1] = -2.0  * val0;
                aval[2] =  1.0  * val0;
                asub[0] = sub_shift + (i + 1) * s1d1CtrlPt_num - 3;
                asub[1] = sub_shift + (i + 1) * s1d1CtrlPt_num - 2;
                asub[2] = sub_shift + (i + 1) * s1d1CtrlPt_num - 1;
                // Next segment's first velocity control point
                aval[3] =  -1.0  * val1;
                aval[4] =   2.0  * val1;
                aval[5] =  -1.0  * val1;
                asub[3] = sub_shift + s1CtrlPt_num + i * s1d1CtrlPt_num;
                asub[4] = sub_shift + s1CtrlPt_num + i * s1d1CtrlPt_num + 1;
                asub[5] = sub_shift + s1CtrlPt_num + i * s1d1CtrlPt_num + 2;

                r = MSK_putarow(task, row_idx, nzi, asub, aval);
                row_idx ++;
            }
            sub_shift += s1CtrlPt_num;
        }

        // Start stacking the objective")
        int min_order_l = std::floor(_minimize_order);
        int min_order_u = std::ceil (_minimize_order);
        int NUMQNZ = 0;
        for (int i = 0; i < segment_num; i++){
            int NUMQ_blk = _traj_order + 1;
            NUMQNZ += 2 * NUMQ_blk * (NUMQ_blk + 1) / 2;//tag: ???
        }
        MSKint32t qsubi[NUMQNZ], qsubj[NUMQNZ];
        double qval[NUMQNZ];

        sub_shift = 0;
        int idx = 0;
        for (int k = 0; k < segment_num; k++){
            double scale_k = _region[k]->t_;
            for (int p = 0; p < 2; p++){
                for (int i = 0; i < s1d1CtrlPt_num; i++){
                    for (int j = 0; j < s1d1CtrlPt_num; j++){
                        if(i >= j){
                            qsubi[idx] = sub_shift + p * s1d1CtrlPt_num + i;
                            qsubj[idx] = sub_shift + p * s1d1CtrlPt_num + j;

                            if(min_order_l == min_order_u)
                                qval[idx] = _MQM(i, j) / pow(scale_k, 2 * min_order_u - 3);
                            else
                                qval[idx] = _MQM(i, j) * ((_minimize_order - min_order_l) / std::pow(scale_k, 2 * min_order_u - 3) +
                                                          (min_order_u - _minimize_order) / std::pow(scale_k, 2 * min_order_l - 3));
                            idx++;
                        }
                    }
                }
            }
            sub_shift += s1CtrlPt_num;
        }

        if(r == MSK_RES_OK)
            r = MSK_putqobj(task, NUMQNZ, qsubi, qsubj, qval);
        if(r == MSK_RES_OK)
            r = MSK_putobjsense(task, MSK_OBJECTIVE_SENSE_MINIMIZE);

        bool solve_ok = false;
        if(r == MSK_RES_OK){
            MSKrescodee trmcode;
            r = MSK_optimizetrm(task, &trmcode);
            MSK_solutionsummary(task, MSK_STREAM_LOG);
            if(r == MSK_RES_OK){
                MSKsolstae solsta;
                MSK_getsolsta(task, MSK_SOL_ITR, &solsta);

                switch(solsta){
                    case MSK_SOL_STA_OPTIMAL:

                    case MSK_SOL_STA_NEAR_OPTIMAL:
                        r = MSK_getxx(task, MSK_SOL_ITR, x_var);
                        r = MSK_getprimalobj(task, MSK_SOL_ITR, &primalobj);
                        _obj = primalobj;
                        solve_ok = true;
                        break;

                    case MSK_SOL_STA_DUAL_INFEAS_CER:

                    case MSK_SOL_STA_PRIM_INFEAS_CER:

                    case MSK_SOL_STA_NEAR_DUAL_INFEAS_CER:

                    case MSK_SOL_STA_NEAR_PRIM_INFEAS_CER:
                        printf("Primal or dual infeasibility certificate found.\n");
                        break;

                    case MSK_SOL_STA_UNKNOWN:
                        printf("The status of the solution could not be determined.\n");
                        //solve_ok = true; // debug
                    break;

                    default:
                        printf("Other solution status.");
                        break;
                }
            }
            else
                std::cout << "Error while optimizing" << std::endl;
        }
        if(r != MSK_RES_OK){
            char symname[MSK_MAX_STR_LEN];
            char desc[MSK_MAX_STR_LEN];

            std::cout << "An error occurred while optimizing" << std::endl;
            MSK_getcodedesc(r, symname, desc);
            printf("Error %s - '%s'\n",symname,desc);
        }
        MSK_deletetask(&task);
        MSK_deleteenv(&env);

        if(!solve_ok){
            std::cout << "solver failed" << std::endl;
            return -1;
        }
        Eigen::VectorXd d_var(ctrlPt_num);
        for (int i = 0; i < ctrlPt_num; i++)
            d_var(i) = x_var[i];

        _PolyCoeff = Eigen::MatrixXd::Zero(segment_num, 3 * (_traj_order + 1));

        int var_shift = 0;
        for (int i = 0; i < segment_num; i++)
            for (int j = 0; j < 2 * n_ploy; j++){
                _PolyCoeff(i, j) = d_var(j + var_shift);
                var_shift += 2 * n_ploy;
            }

        return 1;
    }
}
