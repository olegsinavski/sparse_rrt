/**
 * @file point.cpp
 * 
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2014, Rutgers the State University of New Jersey, New Brunswick  
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 * 
 * Authors: Zakary Littlefield, Kostas Bekris 
 * 
 */

#include "systems/point.hpp"
#include "utilities/random.hpp"
#include <cmath>
#include <assert.h>

#define MIN_X -10
#define MAX_X 10
#define MIN_Y -10
#define MAX_Y 10

#define MIN_V 0
#define MAX_V 10
#define MIN_THETA -3.14
#define MAX_THETA 3.14


bool point_t::propagate(
    const double* start_state, unsigned int state_dimension,
    const double* control, unsigned int control_dimension,
    int num_steps, double* result_state, double integration_step)
{
	temp_state[0] = start_state[0];
	temp_state[1] = start_state[1];
	bool validity = true;
	for(int i=0;i<num_steps;i++)
	{
		temp_state[0] += integration_step*control[0]*cos(control[1]);
		temp_state[1] += integration_step*control[0]*sin(control[1]);
		enforce_bounds();
		validity = validity && valid_state();
	}
	result_state[0] = temp_state[0];
	result_state[1] = temp_state[1];
	return validity;
}

void point_t::enforce_bounds()
{
	if(temp_state[0]<MIN_X)
		temp_state[0]=MIN_X;
	else if(temp_state[0]>MAX_X)
		temp_state[0]=MAX_X;

	if(temp_state[1]<MIN_Y)
		temp_state[1]=MIN_Y;
	else if(temp_state[1]>MAX_Y)
		temp_state[1]=MAX_Y;
}


bool point_t::valid_state()
{
	bool obstacle_collision = false;
	//any obstacles need to be checked here
	for(unsigned i=0;i<obstacles.size() && !obstacle_collision;i++)
	{
		if(	temp_state[0]>obstacles[i].low_x && 
			temp_state[0]<obstacles[i].high_x && 
			temp_state[1]>obstacles[i].low_y && 
			temp_state[1]<obstacles[i].high_y)
		{
			obstacle_collision = true;
		}
	}

	return !obstacle_collision && 
			(temp_state[0]!=MIN_X) &&
			(temp_state[0]!=MAX_X) &&
			(temp_state[1]!=MIN_Y) &&
			(temp_state[1]!=MAX_Y);
}

svg::Point point_t::visualize_point(const double* state, svg::Dimensions dims)
{
	double x = (state[0]-MIN_X)/(MAX_X-MIN_X) * dims.width; 
	double y = (state[1]-MIN_Y)/(MAX_Y-MIN_Y) * dims.height; 
	return svg::Point(x,y);
}

void point_t::visualize_obstacles(svg::Document& doc ,svg::Dimensions dims)
{// doc << Rectangle(Point(-4.5*50+500-175,5*50+500+37.5), 7*50, 1.5*50, Color::Red);
	double temp[2];
	for(unsigned i=0;i<obstacles.size();i++)
	{
		temp[0] = obstacles[i].low_x;
		temp[1] = obstacles[i].high_y;
		doc<<svg::Rectangle(visualize_point(temp,dims), 
							(obstacles[i].high_x-obstacles[i].low_x)/(MAX_X-MIN_X) * dims.width,
							(obstacles[i].high_y-obstacles[i].low_y)/(MAX_Y-MIN_Y) * dims.height,
							svg::Color::Red);
	}
}

std::vector<std::pair<double, double> > point_t::get_state_bounds() {
	return {
			{MIN_X,MAX_X},
			{MIN_Y,MAX_Y}
	};
}


std::vector<std::pair<double, double> > point_t::get_control_bounds() {
	return {
			{MIN_V, MAX_V},
			{MIN_THETA, MAX_THETA},
	};
}


std::vector<bool> point_t::is_circular_topology() {
    return {
            false,
            false
    };
}
