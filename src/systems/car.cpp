/**
 * @file car.cpp
 * 
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2014, Rutgers the State University of New Jersey, New Brunswick  
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 * 
 * Authors: Zakary Littlefield, Kostas Bekris 
 * 
 */

#include "systems/car.hpp"
#include "utilities/random.hpp"


#define _USE_MATH_DEFINES

#include <cmath>

double car_t::distance(double* point1,double* point2)
{
	double val = fabs(point1[2]-point2[2]);
	if(val > M_PI)
		val = 2*M_PI-val;
	return std::sqrt( val * val + (point1[1]-point2[1]) * (point1[1]-point2[1])+(point1[0]-point2[0]) * (point1[0]-point2[0]) );
}

void car_t::random_state(double* state)
{
	state[0] = uniform_random(-10,10);
	state[1] = uniform_random(-10,10);
	state[2] = uniform_random(-M_PI,M_PI);
}

void car_t::random_control(double* control)
{
	control[0] = uniform_random(0,1);
	control[1] = uniform_random(-.5,.5);
}

bool car_t::propagate( double* start_state, double* control, int min_step, int max_step, double* result_state, double& duration )
{
	temp_state[0] = start_state[0]; temp_state[1] = start_state[1];temp_state[2] = start_state[2];

	int num_steps = uniform_int_random(min_step,max_step);
	bool validity = true;
	for(int i=0;i<num_steps;i++)
	{
		double temp0 = temp_state[0];
		double temp1 = temp_state[1];
		double temp2 = temp_state[2];
		temp_state[0] += params::integration_step*cos(temp2)*control[0];
		temp_state[1] += params::integration_step*sin(temp2)*control[0];
		temp_state[2] += params::integration_step*control[1];
		enforce_bounds();
		validity = validity && valid_state();
	}
	result_state[0] = temp_state[0];
	result_state[1] = temp_state[1];
	result_state[2] = temp_state[2];
	duration = num_steps*params::integration_step;
	return validity;
}

void car_t::enforce_bounds()
{
	if(temp_state[0]<-10)
		temp_state[0]=-10;
	else if(temp_state[0]>10)
		temp_state[0]=10;

	if(temp_state[1]<-10)
		temp_state[1]=-10;
	else if(temp_state[1]>10)
		temp_state[1]=10;

	if(temp_state[2]<-M_PI)
		temp_state[2]+=2*M_PI;
	else if(temp_state[2]>M_PI)
		temp_state[2]-=2*M_PI;
}


bool car_t::valid_state()
{
	return 	(temp_state[0]!=-10) &&
			(temp_state[0]!=10) &&
			(temp_state[1]!=-10) &&
			(temp_state[1]!=10);
}

svg::Point car_t::visualize_point(double* state, svg::Dimensions dims)
{
	double x = (state[0]+10)/(20) * dims.width; 
	double y = (state[1]+10)/(20) * dims.height; 
	return svg::Point(x,y);
}