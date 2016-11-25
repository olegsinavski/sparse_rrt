/**
 * @file pendulum.hpp
 * 
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2014, Rutgers the State University of New Jersey, New Brunswick  
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 * 
 * Authors: Zakary Littlefield, Kostas Bekris 
 * 
 */

#ifndef SPARSE_PENDULUM_HPP
#define SPARSE_PENDULUM_HPP

#include "systems/system.hpp"

class pendulum_t : public system_t
{
public:
	pendulum_t()
	{
		state_dimension = 2;
		control_dimension = 1;
		temp_state = new double[state_dimension];
	}
	virtual ~pendulum_t(){}

	/**
	 * @copydoc system_t::distance(double*, double*)
	 */
	virtual double distance(double* point1, double* point2);

	/**
	 * @copydoc system_t::random_state(double*)
	 */
	virtual void random_state(double* state);

	/**
	 * @copydoc system_t::random_control(double*)
	 */
	virtual void random_control(double* control);

	/**
	 * @copydoc system_t::propagate(double*, double*, int, int, double*, double& )
	 */
	virtual bool propagate( double* start_state, double* control, int min_step, int max_step, double* result_state, double& duration );

	/**
	 * @copydoc system_t::enforce_bounds()
	 */
	virtual void enforce_bounds();
	
	/**
	 * @copydoc system_t::valid_state()
	 */
	virtual bool valid_state();

	/**
	 * @copydoc system_t::visualize_point(double*, svg::Dimensions)
	 */
	svg::Point visualize_point(double* state, svg::Dimensions dims);
};


#endif