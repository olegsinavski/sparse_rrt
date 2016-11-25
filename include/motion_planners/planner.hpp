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
	virtual void step() = 0;

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

	/**
	 * @brief Generate an image visualizing the tree.
	 * @details Generate an image visualizing the tree.
	 * 
	 * @param image_counter A subscript for the image file name. Allows for multiple image output.
	 */
	void visualize_tree(int image_counter);

	/**
	 * @brief Generate an image visualizing the nodes in the tree.
	 * @details Generate an image visualizing the nodes in the tree. The nodes will have a grayscale
	 * color corresponding to their relative cost to the maximum in the tree.
	 * 
	 * @param image_counter A subscript for the image file name. Allows for multiple image output.
	 */
	void visualize_nodes(int image_counter);

	/**
	 * @brief Find the maximum cost node in the tree.
	 * @details Find the maximum cost node in the tree.
	 */
	void get_max_cost();

	/** @brief The number of nodes in the tree. */
	unsigned number_of_nodes;

protected:

	/**
	 * @brief Create geometries for visualizing the solution path.
	 * @details Create geometries for visualizing the solution path.
	 * 
	 * @param doc The image storage.
	 * @param dim The size of the image.
	 */
	virtual void visualize_solution_path( svg::Document& doc, svg::Dimensions& dim);

	/**
	 * @brief Create geometries for visualizing the nodes along the solution path.
	 * @details Create geometries for visualizing the nodes along the solution path.
	 * 
	 * @param doc The image storage.
	 * @param dim The size of the image.
	 */
	virtual void visualize_solution_nodes( svg::Document& doc, svg::Dimensions& dim);

	/**
	 * @brief A recursive function for finding the highest cost in the tree.
	 * @details A recursive function for finding the highest cost in the tree.
	 * 
	 * @param node The current node being examined.
	 */
	virtual void get_max_cost(tree_node_t* node);

	/**
	 * @brief Creates a single edge geometry with a node's parent.
	 * @details Creates a single edge geometry with a node's parent.
	 * 
	 * @param node The target node of the edge.
	 * @param doc The image storage.
	 * @param dim The size of the image.
	 */
	virtual void visualize_edge(tree_node_t* node, svg::Document& doc, svg::Dimensions& dim);

	/**
	 * @brief Creates a single node geometry.
	 * @details Creates a single node geometry.
	 * 
	 * @param node The node to visualize.
	 * @param doc The image storage.
	 * @param dim The size of the image.
	 */
	virtual void visualize_node(tree_node_t* node, svg::Document& doc, svg::Dimensions& dim);

 	/**
 	 * @brief The stored solution from previous call to get_solution.
 	 */
    std::vector<tree_node_t*> last_solution_path;

    /**
     * @brief A temporary storage for sorting nodes based on cost.
     */
    std::vector<tree_node_t*> sorted_nodes;

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

	/**
	 * @brief The maximum cost found in the tree.
	 */
	double max_cost;

};


#endif