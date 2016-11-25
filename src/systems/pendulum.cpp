/**
 * @file pendulum.cpp
 * 
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2014, Rutgers the State University of New Jersey, New Brunswick  
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 * 
 * Authors: Zakary Littlefield, Kostas Bekris 
 * 
 */

#include "systems/pendulum.hpp"
#include "utilities/random.hpp"


#define _USE_MATH_DEFINES

#include <cmath>

#define MIN_W -7
#define MAX_W 7

#define MIN_TORQUE -1
#define MAX_TORQUE 1

#define LENGTH 1
#define MASS 1
#define DAMPING .05

double pendulum_t::distance(double* point1,double* point2)
{
	double val = fabs(point1[0]-point2[0]);
	if(val > M_PI)
		val = 2*M_PI-val;
	return std::sqrt( val * val + (point1[1]-point2[1]) * (point1[1]-point2[1]) );
}

void pendulum_t::random_state(double* state)
{
	state[0] = uniform_random(-M_PI,M_PI);
	state[1] = uniform_random(MIN_W,MAX_W);
}

void pendulum_t::random_control(double* control)
{
	control[0] = uniform_random(MIN_TORQUE,MAX_TORQUE);
}

bool pendulum_t::propagate( double* start_state, double* control, int min_step, int max_step, double* result_state, double& duration )
{
	temp_state[0] = start_state[0]; temp_state[1] = start_state[1];
	int num_steps = uniform_int_random(min_step,max_step);
	bool validity = true;
	for(int i=0;i<num_steps;i++)
	{
		double temp0 = temp_state[0];
		double temp1 = temp_state[1];
		temp_state[0] += params::integration_step*temp1;
		temp_state[1] += params::integration_step*
							((control[0] - MASS * (9.81) * LENGTH * cos(temp0)*0.5 
										 - DAMPING * temp1)* 3 / (MASS * LENGTH * LENGTH));
		enforce_bounds();
		validity = validity && valid_state();
	}
	result_state[0] = temp_state[0];
	result_state[1] = temp_state[1];
	duration = num_steps*params::integration_step;
	return validity;
}

void pendulum_t::enforce_bounds()
{
	if(temp_state[0]<-M_PI)
		temp_state[0]+=2*M_PI;
	else if(temp_state[0]>M_PI)
		temp_state[0]-=2*M_PI;

	if(temp_state[1]<MIN_W)
		temp_state[1]=MIN_W;
	else if(temp_state[1]>MAX_W)
		temp_state[1]=MAX_W;
}


bool pendulum_t::valid_state()
{
	return true;
}

svg::Point pendulum_t::visualize_point(double* state, svg::Dimensions dims)
{
	double x = (state[0]+M_PI)/(2*M_PI) * dims.width; 
	double y = (state[1]-MIN_W)/(MAX_W-MIN_W) * dims.height; 
	return svg::Point(x,y);
}