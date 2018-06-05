/**
 * @file system.hpp
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

#ifndef SPARSE_SYSTEM_HPP
#define SPARSE_SYSTEM_HPP
#include <tuple>
#include <vector>

#include "systems/distance_functions.h"


/**
 * A minimal interface for a controllable system.
 * TODO: This currently includes visualization abilities, but it should be moved out
 *
 */
struct system_interface {
    /**
	 * @brief Performs a local propagation using simple numerical integration.
	 * @details Performs a local propagation using simple numerical integration.
	 *
	 * @param start_state The state to start propagating from.
	 * @param control The control to apply for this propagation.
	 * @param min_step The smallest number of simulation steps to execute.
	 * @param max_step The largest number of simulation steps to execute.
	 * @param result_state The result of the propagation.
	 * @param duration The amount of simulation time used.
	 * @return True if this propagation was valid, false if not.
	 */
    virtual bool propagate(
        const double* start_state, unsigned int state_dimension,
        const double* control, unsigned int control_dimension,
        int num_steps, double* result_state, double integration_step) = 0;
    /**
     * @brief Creates a point in image space corresponding to a given state.
     * @details Creates a point in image space corresponding to a given state.
     *
     * @param state The state in the system's space.
     * @param dims The size of the destination image.
     *
     * @return A point in image space.
     */
    virtual std::tuple<double, double> visualize_point(const double* state, unsigned int state_dimension) const = 0;

    /**
     * @brief Visualize the obstacles for this system.
     * @details Visualize the obstacles for this system.
     *
     * @param doc The image storage.
     * @param dims The image size.
     */
    virtual std::string visualize_obstacles(int image_width, int image_height) const
    {
    	return "";
    }
};


/**
 * @brief A base class for plannable systems.
 * @details A base class for plannable systems. This class implements core functionality
 * related to creating state and control memory, propagations, obstacles, random states
 * and controls, and visualizing points.
 * 
 */
class system_t: public system_interface
{
public: 
	system_t(){}
	virtual ~system_t(){}

    /**
	 * @brief The dimensionality of the state space.
	 * @details The dimensionality of the state space.
	 *
	 * @return The dimensionality of the state space.
	 */
	unsigned get_state_dimension()
	{
		return state_dimension;
	}

	/**
	 * @brief The dimensionality of the control space.
	 * @details The dimensionality of the control space.
	 *
	 * @return The dimensionality of the control space.
	 */
	unsigned get_control_dimension()
	{
		return control_dimension;
	}

    /**
     * @brief Array of tuples (min, max) indicating the bounds of the state space
     * @details Array of tuples (min, max) indicating the bounds of the state space
     *
     */
    virtual std::vector<std::pair<double, double>> get_state_bounds() const = 0;

      /**
     * @brief Array of tuples (min, max) indicating the bounds of the control space
     * @details Array of tuples (min, max) indicating the bounds of the control space
     *
     */
    virtual std::vector<std::pair<double, double>> get_control_bounds() const = 0;

    /**
     * @brief Array of flags indicating that a degree of freedom has circular topology
     * @details Array of flags indicating that a degree of freedom has circular topology
     *
     */
	virtual std::vector<bool> is_circular_topology() const = 0;

protected:

	/**
	 * @brief Enforce bounds on the state space.
	 * @details Enforce bounds on the state space.
	 */
	virtual void enforce_bounds() = 0;

	/**
	 * @brief Determine if the current state is in collision or out of bounds.
	 * @details Determine if the current state is in collision or out of bounds.
	 * @return True if this state was valid, false if not.
	 */
	virtual bool valid_state() = 0;

	/**
	 * @brief The dimensionality of the state space.
	 */
	unsigned state_dimension;

	/**
	 * @brief The dimensionality of the control space.
	 */
	unsigned control_dimension;

	/**
	 * @brief Intermediate storage for propagation.
	 */
	double* temp_state;

};

#endif