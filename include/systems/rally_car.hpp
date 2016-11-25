/**
 * @file rally_car.hpp
 * 
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2014, Rutgers the State University of New Jersey, New Brunswick  
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 * 
 * Authors: Zakary Littlefield, Kostas Bekris 
 * 
 */

#ifndef SPARSE_RALLY_CAR_HPP
#define SPARSE_RALLY_CAR_HPP


#include "systems/system.hpp"
#include "systems/point.hpp"

class rally_car_t : public system_t
{
public:
	rally_car_t()
	{
		state_dimension = 8;
		control_dimension = 3;
		temp_state = new double[state_dimension];
		deriv = new double[state_dimension];

		obstacles.push_back(Rectangle_t(   0,  20.5,    42,  1,true));
		obstacles.push_back(Rectangle_t(  20.5,  -5.5,   1, 53,true));
		obstacles.push_back(Rectangle_t(  9.5,  -16,   1, 32,true));
		obstacles.push_back(Rectangle_t(  15,  -32.5,   12, 1,true));
		obstacles.push_back(Rectangle_t(  -9.5,  -16,   1, 32,true));
		obstacles.push_back(Rectangle_t(  -20.5,  -5.5,   1, 53,true));

	}
	virtual ~rally_car_t(){}

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
	
	/**
	 * @copydoc system_t::visualize_obstacles(svg::Document&, svg::Dimensions)
	 */
    virtual void visualize_obstacles(svg::Document& doc ,svg::Dimensions dims);

protected:
	double* deriv;
	void update_derivative(double* control);
	std::vector<Rectangle_t> obstacles;
};


#endif