/**
 * @file system.hpp
 * 
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2014, Rutgers the State University of New Jersey, New Brunswick  
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 * 
 * Authors: Zakary Littlefield, Kostas Bekris 
 * 
 */

#ifndef SPARSE_SYSTEM_HPP
#define SPARSE_SYSTEM_HPP

#include "image_creation/svg_image.hpp"
#include "utilities/parameter_reader.hpp"

/**
 * @brief A base class for plannable systems.
 * @details A base class for plannable systems. This class implements core functionality
 * related to creating state and control memory, propagations, obstacles, random states
 * and controls, and visualizing points.
 * 
 */
class system_t
{
public: 
	system_t(){}
	virtual ~system_t(){}

	unsigned get_state_dimension()
	{
		return state_dimension;
	}
	unsigned get_control_dimension()
	{
		return control_dimension;
	}

	/**
	 * @brief Performs a random sampling for a new state.
	 * @details Performs a random sampling for a new state.
	 * 
	 * @param state The state to modify with random values.
	 */
	virtual void random_state(double* state) = 0;

	/**
	 * @brief Performs a random sampling for a new control.
	 * @details Performs a random sampling for a new control.
	 * 
	 * @param control The control to modify with random values.
	 */
	virtual void random_control(double* control) = 0;

	/**
	 * @brief Finds the distance between two states.
	 * @details Finds the distance between two states.
	 * 
	 * @param point1 First point.
	 * @param point2 Second point.
	 * 
	 * @return The distance between point1 and point2.
	 */
	virtual double distance(double* point1, double* point2) = 0;

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
    virtual bool propagate( double* start_state, double* control, int min_step, int max_step, double* result_state, double& duration, double integration_step) = 0;

    /**
     * @brief Creates a point in image space corresponding to a given state.
     * @details Creates a point in image space corresponding to a given state.
     * 
     * @param state The state in the system's space.
     * @param dims The size of the destination image.
     * 
     * @return A point in image space.
     */
    virtual svg::Point visualize_point(double* state,svg::Dimensions dims) = 0;

    /**
     * @brief Visualize the obstacles for this system.
     * @details Visualize the obstacles for this system.
     * 
     * @param doc The image storage.
     * @param dims The image size.
     */
    virtual void visualize_obstacles(svg::Document& doc ,svg::Dimensions dims)
    {
    	return;
    }

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
	 * @brief The size of the state space.
	 */
	unsigned state_dimension;

	/**
	 * @brief The size of the control space.
	 */
	unsigned control_dimension;

	/**
	 * @brief Intermediate storage for propagation.
	 */
	double* temp_state;

};

#endif