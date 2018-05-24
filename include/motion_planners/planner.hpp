/**
 * @file planner.hpp
 * 
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2014, Rutgers the State University of New Jersey, New Brunswick  
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 * 
 * Authors: Zakary Littlefield, Kostas Bekris 
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
	 * @param in_system The system this planner will plan for.
	 */
	planner_t(const std::vector<std::pair<double, double> >& a_state_bounds,
              const std::vector<std::pair<double, double> >& a_control_bounds,
              std::function<double(const double*, const double*)> distance_function,
              unsigned int random_seed
    )
        : state_dimension(a_state_bounds.size())
        , state_bounds(a_state_bounds)
        , control_dimension(a_control_bounds.size())
        , control_bounds(a_control_bounds)
        , distance(distance_function)
        , start_state(this->alloc_state_point())
        , goal_state(this->alloc_state_point())
        , number_of_nodes(0)
        , random_generator(random_seed)
    {
	}
	virtual ~planner_t()
	{
        dealloc_state_point(&start_state);
        dealloc_state_point(&goal_state);
	}

	/**
	 * @brief Perform any initialization tasks required before calling step().
	 * @details Perform any initialization tasks required before calling step().
	 */
	virtual void setup_planning() = 0;

	/**
	 * @brief Get the solution path.
	 * @details Query the tree structure for the solution plan for this given system.
	 * 
	 * @param controls The list of controls and durations which comprise the solution.
	 */
	virtual void get_solution(std::vector<std::vector<double>>& solution_path, std::vector<std::vector<double>>& controls, std::vector<double>& costs) = 0;

	/**
	 * @brief Perform an iteration of a motion planning algorithm.
	 * @details Perform an iteration of a motion planning algorithm.
	 */
	virtual void step(system_t* system, int min_time_steps, int max_time_steps, double integration_step) = 0;

	/**
	 * @brief Set the start and goal state for the planner.
	 * @details Set the start and goal state for the planner.
	 *
	 * @param in_start The start state.
	 * @param in_goal The goal state
	 * @param in_radius The radial size of the goal region centered at in_goal.
	 */
	void set_start_goal_state(const double* in_start, const double* in_goal,double in_radius) {
        this->copy_state_point(start_state, in_start);
        this->copy_state_point(goal_state,in_goal);
        goal_radius = in_radius;
    }


	/** @brief The number of nodes in the tree. */
	unsigned number_of_nodes;

	tree_node_t* get_root() { return this->root; }

    /**
	 * @brief Copies one state into another.
	 * @details Copies one state into another.
	 *
	 * @param destination The destination memory.
	 * @param source The point to copy.
	 */
	void copy_state_point(double* destination, const double* source) const
	{
		for(unsigned i=0;i<this->state_dimension;i++)
			destination[i] = source[i];
	}

	/**
	 * @brief Allocates a double array representing a state of this system.
	 * @details Allocates a double array representing a state of this system.
	 * @return Allocated memory for a state.
	 */
	double* alloc_state_point()
	{
	    try {
		    return new double[this->state_dimension];
		} catch (const std::bad_alloc& e) {
		    std::cout << "alloc_state_point (dims=" << this->state_dimension << ") failed: " << e.what() << '\n';
		    return NULL;
		}
	}
	void dealloc_state_point(double** state_point)
	{
	    assert( *state_point != NULL );
		delete[] *state_point;
		*state_point = NULL;
	}

	/**
	 * @brief Allocates a double array representing a control of this system.
	 * @details Allocates a double array representing a control of this system.
	 * @return Allocated memory for a control.
	 */
	double* alloc_control_point()
	{
		return new double[this->control_dimension];
	}

	/**
	 * @brief Copies one control into another.
	 * @details Copies one control into another.
	 *
	 * @param destination The destination memory.
	 * @param source The control to copy.
	 */
	void copy_control_point(double* destination, const double* source)
	{
		for(unsigned i=0;i<this->control_dimension;i++)
			destination[i] = source[i];
	}

	/**
	 * @brief Performs a random sampling for a new state.
	 * @details Performs a random sampling for a new state.
	 *
	 * @param state The state to modify with random values.
	 */
	void random_state(double* state)
	{
		for (int i = 0; i < this->state_bounds.size(); ++i) {
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
        for (int i = 0; i < this->control_bounds.size(); ++i) {
            control[i] = this->random_generator.uniform_random(this->control_bounds[i].first, this->control_bounds[i].second);
        }
	}

	double* get_start_state() {return this->start_state;};
    double* get_goal_state() {return this->goal_state;};

protected:

    unsigned int state_dimension;
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

    std::vector<std::pair<double, double> > state_bounds;
    std::vector<std::pair<double, double> > control_bounds;

    std::function<double(const double*, const double*)> distance;

	RandomGenerator random_generator;
};


#endif