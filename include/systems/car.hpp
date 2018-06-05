/**
 * @file car.hpp
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
	virtual ~car_t(){delete temp_state;}

    /**
	 * @copydoc system_t::propagate()
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
	 * @copydoc system_t::visualize_point()
	 */
	std::tuple<double, double> visualize_point(const double* state, unsigned int state_dimension) const override;

	/**
	 * @copydoc system_t::get_state_bounds()
	 */
	virtual std::vector<std::pair<double, double> > get_state_bounds() const;

	/**
	 * @copydoc system_t::get_control_bounds()
	 */
    virtual std::vector<std::pair<double, double> > get_control_bounds() const;

    /**
	 * @copydoc system_t::is_circular_topology()
	 */
	std::vector<bool> is_circular_topology() const override;

};


#endif