/**
 * @file car.cpp
 *
 * @copyright Software License Agreement (BSD License)
 * Original work Copyright (c) 2014, Rutgers the State University of New Jersey, New Brunswick
 * Modified work Copyright 2017 Oleg Y. Sinyavskiy
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 *
 * Original authors: Zakary Littlefield, Kostas Bekris
 * Modifications by: Oleg Y. Sinyavskiy
 * 
 */

#include "systems/car.hpp"
#include "utilities/random.hpp"


#define _USE_MATH_DEFINES

#include <cmath>


bool car_t::propagate(
    const double* start_state, unsigned int state_dimension,
    const double* control, unsigned int control_dimension,
    int num_steps, double* result_state, double integration_step)
{
	temp_state[0] = start_state[0]; temp_state[1] = start_state[1];temp_state[2] = start_state[2];

	bool validity = true;
	for(int i=0;i<num_steps;i++)
	{
		double temp2 = temp_state[2];
		temp_state[0] += integration_step*cos(temp2)*control[0];
		temp_state[1] += integration_step*sin(temp2)*control[0];
		temp_state[2] += integration_step*control[1];
		enforce_bounds();
		validity = validity && valid_state();
	}
	result_state[0] = temp_state[0];
	result_state[1] = temp_state[1];
	result_state[2] = temp_state[2];
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

std::tuple<double, double> car_t::visualize_point(const double* state, unsigned int state_dimension) const
{
	double x = (state[0]+10)/(20);
	double y = (state[1]+10)/(20);
	return std::make_tuple(x, y);
}

std::vector<std::pair<double, double> > car_t::get_state_bounds() const {
	return {
        {-10,10},
        {-10,10},
        {-M_PI,M_PI},
	};
}


std::vector<std::pair<double, double> > car_t::get_control_bounds() const {
    return {
            {0, 1},
            {-.5,.5},
    };
}

std::vector<bool> car_t::is_circular_topology() const {
	return {
			false,
			false,
			true
	};
}
