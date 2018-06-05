/**
 * @file planner.hpp
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

#ifndef SPARSE_PLANNER_HPP
#define SPARSE_PLANNER_HPP

#include <vector>

#include "systems/system.hpp"
#include "nearest_neighbors/graph_nearest_neighbors.hpp"
#include "motion_planners/tree_node.hpp"
#include "utilities/random.hpp"

/**
 * @brief The base class for motion planners.
 * @details The base class for motion planners. This class provides 
 * methods for visualizing the tree structures produced by motion
 * planners, in addition to initialization functions.
 */
class planner_t
{
public: 
	/**
	 * @brief Planner Constructor
	 * @details Planner Constructor
	 * 
	 * @param in_start The start state.
	 * @param in_goal The goal state
	 * @param in_radius The radial size of the goal region centered at in_goal.
	 * @param a_state_bounds A vector with boundaries of the state space (min and max)
	 * @param a_control_bounds A vector with boundaries of the control space (min and max)
	 * @param distance_function Function that returns distance between two state space points
	 * @param random_seed The seed for the random generator
	 */
	planner_t(
	    const double* in_start, const double* in_goal,
	    double in_radius,
	    const std::vector<std::pair<double, double> >& a_state_bounds,
        const std::vector<std::pair<double, double> >& a_control_bounds,
        std::function<double(const double*, const double*, unsigned int)> distance_function,
        unsigned int random_seed
    )
        : state_dimension(a_state_bounds.size())
        , control_dimension(a_control_bounds.size())
        , root(nullptr)
        , start_state(new double[this->state_dimension])
        , goal_state(new double[this->state_dimension])
        , goal_radius(in_radius)
        , state_bounds(a_state_bounds)
        , control_bounds(a_control_bounds)
        , distance(distance_function)
        , random_generator(random_seed)
        , number_of_nodes(0)
    {
        std::copy(in_start, in_start + this->state_dimension, start_state);
	    std::copy(in_goal, in_goal + this->state_dimension, goal_state);
	}

	virtual ~planner_t()
	{
	    delete[] start_state;
	    delete[] goal_state;
	}

	/**
	 * @brief Get the solution path.
	 * @details Query the tree structure for the solution plan for this given system.
	 *
	 * @param solution_path The list of state pointswhich comprise the solution.
	 * @param controls The list of controls which comprise the solution.
	 * @param costs The list of costs of the edges which comprise the solution.
	 */
	virtual void get_solution(std::vector<std::vector<double>>& solution_path, std::vector<std::vector<double>>& controls, std::vector<double>& costs) = 0;

	/**
	 * @brief Perform an iteration of a motion planning algorithm.
	 * @details Perform an iteration of a motion planning algorithm.
	 *
	 * @param system_interface System object that has to be integrated under planner control
	 * @param min_time_steps Minimum number of control steps for the system
	 * @param max_time_steps Maximum number of control steps for the system
	 * @param integration_step Integration step in seconds to integrate the system
	 */
	virtual void step(system_interface* system, int min_time_steps, int max_time_steps, double integration_step) = 0;

    /**
	 * @brief Return the root of the planning tree
	 * @details Return the root of the planning tree
	 *
	 * @return root of the planning tree
	 */
	tree_node_t* get_root() { return this->root; }

	/**
	 * @brief Performs a random sampling for a new state.
	 * @details Performs a random sampling for a new state.
	 *
	 * @param state The state to modify with random values.
	 */
	void random_state(double* state)
	{
		for (unsigned int i =0; i < this->state_bounds.size(); ++i) {
            state[i] = this->random_generator.uniform_random(this->state_bounds[i].first, this->state_bounds[i].second);
        }
	}

	/**
	 * @brief Performs a random sampling for a new control.
	 * @details Performs a random sampling for a new control.
	 *
	 * @param control The control to modify with random values.
	 */
	void random_control(double* control)
	{
        for (unsigned int i =0; i < this->control_bounds.size(); ++i) {
            control[i] = this->random_generator.uniform_random(this->control_bounds[i].first, this->control_bounds[i].second);
        }
	}

    /**
	 * @brief Return start state
	 * @details Return start state
	 *
	 * @return start state
	 */
	double* get_start_state() {return this->start_state;};
	/**
	 * @brief Return goal state
	 * @details Return goal state
	 *
	 * @return goal state
	 */
    double* get_goal_state() {return this->goal_state;};

    /**
	 * @brief Return dimensionality of the state space
	 * @details Return dimensionality of the state space
	 *
	 * @return dimensionality of the state space
	 */
    unsigned int get_state_dimension() const {return this->state_dimension;};
    /**
	 * @brief Return dimensionality of the control space
	 * @details Return dimensionality of the control space
	 *
	 * @return dimensionality of the control space
	 */
    unsigned int get_control_dimension() const {return this->control_dimension;};

    /**
	 * @brief Return current number of nodes in the planning tree
	 * @details Return current number of nodes in the planning tree
	 *
	 * @return current number of nodes in the planning tree
	 */
    unsigned int get_number_of_nodes() const {return this->number_of_nodes;};

protected:

    /**
     * @brief Dimensionality of the state space
     */
    unsigned int state_dimension;

     /**
     * @brief Dimensionality of the control space
     */
	unsigned int control_dimension;

    /**
     * @brief The tree of the motion planner starts here.
     */
	tree_node_t* root;

	/**
	 * @brief The start state of the motion planning query.
	 */
	double* start_state;

	/**
	 * @brief The goal state of the motion planning query.
	 */
	double* goal_state;

	/**
	 * @brief The size of the spherical goal region around the goal state.
	 */
	double goal_radius;

    /**
     * @brief Boundaries of the state space
     */
    std::vector<std::pair<double, double> > state_bounds;
    /**
     * @brief Boundaries of the control space
     */
    std::vector<std::pair<double, double> > control_bounds;

    /**
     * @brief Distance function for the state space
     */
    std::function<double(const double*, const double*, unsigned int)> distance;

    /**
     * @brief Random number generator for the planner
     */
	RandomGenerator random_generator;

	/** @brief The number of nodes in the tree. */
	unsigned number_of_nodes;
};


#endif