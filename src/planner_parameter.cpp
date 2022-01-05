#include "bazier_traj/planner_parameter.h"

double planner::PlannerParameter::_x_size = 50.0;
double planner::PlannerParameter::_y_size = 50.0;
double planner::PlannerParameter::_z_size = 5.0;
double planner::PlannerParameter::_x_local_size = 22.0;
double planner::PlannerParameter::_y_local_size = 22.0;
double planner::PlannerParameter::_z_local_size = 8.0;
double planner::PlannerParameter::_MAX_VEL = 2.0;
double planner::PlannerParameter::_MAX_ACC = 2.0;
double planner::PlannerParameter::_resolution = 0.5;
double planner::PlannerParameter::_inv_resolution = 2.0;
double planner::PlannerParameter::_cloud_margin = 0.2;

char   planner::PlannerParameter::_robot_model = CUBE_;
double planner::PlannerParameter::_radius = 0.5;
double planner::PlannerParameter::_model_size_x = 1.0;
double planner::PlannerParameter::_model_size_y = 1.0;
double planner::PlannerParameter::_model_size_z = 1.0;
int planner::PlannerParameter::_max_inflate_iter = 100;
int planner::PlannerParameter::_step_length = 2;
int planner::PlannerParameter::_max_x_id = planner::PlannerParameter::_x_local_size * planner::PlannerParameter::_inv_resolution;
int planner::PlannerParameter::_max_y_id = planner::PlannerParameter::_y_local_size * planner::PlannerParameter::_inv_resolution;
int planner::PlannerParameter::_max_z_id = planner::PlannerParameter::_z_local_size * planner::PlannerParameter::_inv_resolution;
