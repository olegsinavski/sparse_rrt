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

	virtual bool propagate( double* start_state, double* control, int num_steps, double* result_state, double integration_step);

	virtual void enforce_bounds();
	
	virtual bool valid_state();

	svg::Point visualize_point(const double* state, svg::Dimensions dims);

	virtual std::vector<std::pair<double, double> > get_state_bounds();
    virtual std::vector<std::pair<double, double> > get_control_bounds();
	std::vector<bool> is_circular_topology() override;

};


#endif