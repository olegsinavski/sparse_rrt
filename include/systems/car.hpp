/**
 * @file car.hpp
 * 
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2014, Rutgers the State University of New Jersey, New Brunswick  
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 * 
 * Authors: Zakary Littlefield, Kostas Bekris 
 * 
 */

#ifndef SPARSE_CAR_HPP
#define SPARSE_CAR_HPP

#include "systems/system.hpp"

class car_t : public system_t
{
public:
	car_t()
	{
		state_dimension = 3;
		control_dimension = 2;
		temp_state = new double[state_dimension];
	}
	virtual ~car_t(){}

	virtual double distance(double* point1, double* point2);

	virtual void random_state(double* state);

	virtual void random_control(double* control);

	virtual bool propagate( double* start_state, double* control, int min_step, int max_step, double* result_state, double& duration );

	virtual void enforce_bounds();
	
	virtual bool valid_state();

	svg::Point visualize_point(double* state, svg::Dimensions dims);
};


#endif