/**
 * @file run.cpp
 * 
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2014, Rutgers the State University of New Jersey, New Brunswick  
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 * 
 * Authors: Zakary Littlefield, Kostas Bekris 
 * 
 */

#include "utilities/parameter_reader.hpp"
#include "utilities/condition_check.hpp"
#include "utilities/random.hpp"
#include "utilities/timer.hpp"

#include "systems/pendulum.hpp"
#include "systems/point.hpp"
#include "systems/car.hpp"
#include "systems/rally_car.hpp"
#include "systems/cart_pole.hpp"
#include "systems/two_link_acrobot.hpp"
#include "motion_planners/sst.hpp"
#include "motion_planners/rrt.hpp"

#include <iostream>

int main(int ac, char* av[])
{
	read_parameters(ac,av);
	//****************After reading in from input, we need to instantiate classes
	init_random(params::random_seed);
	system_t* system;
	if(params::system=="point")
	{
		system = new point_t();
	}
	else if(params::system=="pendulum")
	{
		system = new pendulum_t();
	}
	else if(params::system=="car")
	{
		system = new car_t();
	}
	else if(params::system=="rally_car")
	{
		system = new rally_car_t();
	}
	else if(params::system=="cart_pole")
	{
		system = new cart_pole_t();
	}
	else if(params::system=="two_link_acrobot")
	{
		system = new two_link_acrobot_t();
	}

// parameters
//	std::string planner_name = "sst";
//	std::string system_name = "point";
//
//	std::string stopping_type = "iterations";
//	double stopping_check = 410000;
//
//	std::string stats_type = "time";
//	double stats_check_value = 0;
//
//	int random_seed = 0;
//	double start_state[] = {0., 0.};
//	double goal_state[] = {9., 9.};
//	double goal_radius = 0.5;
//
//	// The radius for BestNear (delta_n in the WAFR paper)
//	double sst_delta_near=.4;
//	// The radius for sparsification (delta_s in the WAFR paper)
//	double sst_delta_drain=.2;
//
//	bool intermediate_visualization = false;
//
//	int min_time_steps = 20;
//	int max_time_steps = 200;
//	double integration_step = 0.002;


    planner_t* planner;
    if(params::planner=="rrt")
    {
        planner = new rrt_t(system);
    }
    else if(params::planner=="sst")
    {
        planner = new sst_t(
                system->get_state_bounds(), system->get_control_bounds(),
                euclidian_distance(system->is_circular_topology()),
                random_seed,
                sst_delta_near, sst_delta_drain);
    }

	condition_check_t checker(stopping_type, stopping_check);
	condition_check_t* stats_check=NULL;
	if(stats_check_value!=0)
	{
		stats_check = new condition_check_t(stats_type, stats_check_value);
	}

	checker.reset();
	std::cout<<"Starting the planner: "<<planner_name<<" for the system: "<<system_name<<std::endl;

	int count = 0;
	bool execution_done = false;
	bool stats_print = false;
	while(true)
	{
		do
		{
			planner->step(system, min_time_steps, max_time_steps, integration_step);
			execution_done = checker.check();
			if (stats_check != NULL) {
				stats_print = stats_check->check();
			}
		}
		while(!execution_done && !stats_print);

		std::vector<std::pair<double*,double> > controls;
		planner->get_solution(controls);
		double solution_cost = 0;
		for(unsigned i=0;i<controls.size();i++)
		{
			solution_cost+=controls[i].second;
		}
		std::cout<<"Time: "<<checker.time()<<" Iterations: "<<checker.iterations()<<" Nodes: "<<planner->number_of_nodes<<" Solution Quality: " <<solution_cost<<std::endl ;
		stats_print = false;
		if(intermediate_visualization || execution_done)
		{
			//this->visualize(count, system);
			count++;
		}
		if (stats_check != NULL) {
			stats_check->reset();
		}

		if (execution_done)
		{
			break;
		}
	}
	std::cout<<"Done planning."<<std::endl;

}