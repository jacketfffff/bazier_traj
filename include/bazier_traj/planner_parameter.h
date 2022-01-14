#ifndef PLANNER_PARAMETER_H
#define PLANNER_PARAMETER_H

#include <iostream>

#define CUBE_ 'A'
#define CYLINDER_ 'B'

namespace planner{
class PlannerParameter{
    public:
        // tag: local map
        // tag: _()_local_size : size of local_map
        static double _x_size, _y_size, _z_size;
        static double _x_local_size, _y_local_size, _z_local_size;
        static double _MAX_VEL, _MAX_ACC;
        static double _resolution, _inv_resolution;
        static double _cloud_margin;
        // tag: robot collision model
        static double _radius;
        static double _model_size_x, _model_size_y, _model_size_z;
        static char _robot_model;
        //tag: region segmentation
        static int _max_inflate_iter;
        static int _step_length;
        static int _max_y_id;
        static int _max_x_id;
        static int _max_z_id;
        static double _start_x, _start_y, _start_z;
        static double _end_x, _end_y, _end_z;
};
};

#endif