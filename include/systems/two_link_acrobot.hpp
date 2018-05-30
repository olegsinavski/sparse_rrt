/**
 * @file two_link_acrobot.hpp
 * 
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2014, Rutgers the State University of New Jersey, New Brunswick  
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 * 
 * Authors: Zakary Littlefield, Kostas Bekris 
 * 
 */

#ifndef SPARSE_TWO_LINK_ACROBOT_HPP
#define SPARSE_TWO_LINK_ACROBOT_HPP


#include "systems/system.hpp"

class two_link_acrobot_t : public system_t
{
public:
	two_link_acrobot_t()
	{
		state_dimension = 4;
		control_dimension = 1;
		temp_state = new double[state_dimension];
		deriv = new double[state_dimension];

	}
	virtual ~two_link_acrobot_t(){}

	/**
	 * @copydoc system_t::distance(double*, double*)
	 */
	static double distance(const double* point1, const double* point2);

	/**
	 * @copydoc system_t::propagate(double*, double*, int, int, double*, double& )
	 */
	virtual bool propagate(
		const double* start_state, unsigned int state_dimension,
        const double* control, unsigned int control_dimension,
	    int num_steps, double* result_state, double integration_step);

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
	svg::Point visualize_point(const double* state, svg::Dimensions dims);

	std::vector<std::pair<double, double>> get_state_bounds() override;
	std::vector<std::pair<double, double>> get_control_bounds() override;
	std::vector<bool> is_circular_topology() override;
	
protected:
	double* deriv;
	void update_derivative(const double* control);

};


#endif