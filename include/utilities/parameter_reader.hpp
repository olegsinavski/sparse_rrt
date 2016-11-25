/**
 * @file parameter_reader.hpp
 * 
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2014, Rutgers the State University of New Jersey, New Brunswick  
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 * 
 * Authors: Zakary Littlefield, Kostas Bekris 
 * 
 */

#ifndef SPARSE_PARAMETER_READER
#define SPARSE_PARAMETER_READER

#include <string>

namespace params
{
	extern double integration_step;
	extern std::string stopping_type;
	extern double stopping_check;
	extern std::string stats_type;
	extern double stats_check;
	extern bool intermediate_visualization;
	extern unsigned min_time_steps;
	extern unsigned max_time_steps;
	extern int random_seed;
	extern double sst_delta_near;
	extern double sst_delta_drain;
	extern std::string planner;
	extern std::string system;
	extern double* start_state;
	extern double* goal_state;
	extern double goal_radius;


	//Parameters for image output.
	extern double tree_line_width;
	extern double solution_line_width;
	extern int image_width;
	extern int image_height;
	extern double node_diameter;
	extern double solution_node_diameter;
}

void read_parameters(int ac, char* av[]);


#endif