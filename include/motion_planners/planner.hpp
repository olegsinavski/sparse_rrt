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

#include "utilities/parameter_reader.hpp"
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
	planner_t(system_t* in_system)
	{
		system = in_system;
		start_state = NULL;
		goal_state = NULL;
		number_of_nodes=0;
		state_dimension = in_system->get_state_dimension();
        state_bounds = in_system->get_state_bounds();
        assert(state_bounds.size() == state_dimension);
		control_dimension = in_system->get_control_dimension();
        control_bounds = in_system->get_control_bounds();
        assert(control_bounds.size() == control_dimension);
	}
	virtual ~planner_t()
	{

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
	virtual void get_solution(std::vector<std::pair<double*,double> >& controls) = 0;

	/**
	 * @brief Perform an iteration of a motion planning algorithm.
	 * @details Perform an iteration of a motion planning algorithm.
	 */
	virtual void step(int min_time_steps, int max_time_steps, double integration_step) = 0;

	/**
	 * @brief Set the start state for the planner.
	 * @details Set the start state for the planner.
	 * 
	 * @param in_start The start state.
	 */
	void set_start_state(double* in_start) {
	    if(start_state==NULL)
		    start_state = this->alloc_state_point();
        this->copy_state_point(start_state, in_start);
	}

	/**
	 * @brief Set the goal state for the planner.
	 * @details Set the goal state for the planner.
	 * 
	 * @param in_goal The goal state
	 * @param in_radius The radial size of the goal region centered at in_goal.
	 */
	void set_goal_state(double* in_goal,double in_radius) {
        if(goal_state==NULL)
            goal_state = this->alloc_state_point();
        this->copy_state_point(goal_state,in_goal);
        goal_radius = in_radius;
    }


	/** @brief The number of nodes in the tree. */
	unsigned number_of_nodes;

	tree_node_t* get_root() { return this->root; }
    const std::vector<tree_node_t*>& get_last_solution_path() { return this->last_solution_path; }

    /**
	 * @brief Copies one state into another.
	 * @details Copies one state into another.
	 *
	 * @param destination The destination memory.
	 * @param source The point to copy.
	 */
	void copy_state_point(double* destination, double* source)
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
		return new double[this->state_dimension];
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
	void copy_control_point(double* destination, double* source)
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
            state[i] = uniform_random(this->state_bounds[i].first, this->state_bounds[i].second);
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
        for (int i = 0; i < this->state_bounds.size(); ++i) {
            control[i] = uniform_random(this->control_bounds[i].first, this->control_bounds[i].second);
        }
	}

protected:

 	/**
 	 * @brief The stored solution from previous call to get_solution.
 	 */
    std::vector<tree_node_t*> last_solution_path;

    /**
     * @brief The tree of the motion planner starts here.
     */
	tree_node_t* root;

	/**
	 * @brief The nearest neighbor data structure.
	 */
	graph_nearest_neighbors_t* metric;

	/**
	 * @brief The system being planned for.
	 */
	system_t* system;

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

	unsigned int state_dimension;
	unsigned int control_dimension;

    std::vector<std::pair<double, double> > state_bounds;
    std::vector<std::pair<double, double> > control_bounds;

};


#endif