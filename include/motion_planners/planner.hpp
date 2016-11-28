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
	virtual void set_start_state(double* in_start);

	/**
	 * @brief Set the goal state for the planner.
	 * @details Set the goal state for the planner.
	 * 
	 * @param in_goal The goal state
	 * @param in_radius The radial size of the goal region centered at in_goal.
	 */
	virtual void set_goal_state(double* in_goal,double in_radius);

	/** @brief The number of nodes in the tree. */
	unsigned number_of_nodes;

	tree_node_t* get_root() { return this->root; }
    const std::vector<tree_node_t*>& get_last_solution_path() { return this->last_solution_path; }

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

};


#endif