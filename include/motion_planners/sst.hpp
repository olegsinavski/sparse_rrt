/**
 * @file sst.hpp
 * 
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2014, Rutgers the State University of New Jersey, New Brunswick  
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 * 
 * Authors: Zakary Littlefield, Kostas Bekris 
 * 
 */

#ifndef SPARSE_PLANNER_SST_HPP
#define SPARSE_PLANNER_SST_HPP

#include "systems/system.hpp"
#include "motion_planners/planner.hpp"

class sample_node_t;

/**
 * @brief A special storage node for SST.
 * @details A special storage node for SST.
 */
class sst_node_t : public tree_node_t
{
public:
	sst_node_t() : tree_node_t()
	{
		inactive = false;
		witness = NULL;
	}
	/**
	 * A flag for inclusion in the metric.
	 */
	bool inactive;

	sample_node_t* witness;

};

/**
 * @brief A special storage node for witness nodes in SST.
 * @details A special storage node for witness nodes in SST.
 * 
 */
class sample_node_t : public tree_node_t
{
public:
	sample_node_t() : tree_node_t()
	{
		rep = NULL;
	}
	/**
	 * The node that represents this sample.
	 */
	sst_node_t* rep;

};


/**
 * @brief The motion planning algorithm SST (Stable Sparse-RRT)
 * @details The motion planning algorithm SST (Stable Sparse-RRT)
 */
class sst_t : public planner_t
{
public:
	/**
	 * @copydoc planner_t::planner_t()
	 */
	sst_t(const std::vector<std::pair<double, double> >& a_state_bounds,
		  const std::vector<std::pair<double, double> >& a_control_bounds,
		  std::function<double(double*, double*)> distance_function,
		  double delta_near, double delta_drain)
		: planner_t(a_state_bounds, a_control_bounds, distance_function)
	    , sst_delta_near(delta_near)
	    , sst_delta_drain(delta_drain)
	{

	}
	virtual ~sst_t(){}

	/**
	 * @copydoc planner_t::setup_planning()
	 */
	virtual void setup_planning();

	/**
	 * @copydoc planner_t::get_solution(std::vector<std::pair<double*,double> >&)
	 */
	virtual void get_solution(std::vector<std::pair<double*,double> >& controls);
	
	/**
	 * @copydoc planner_t::step()
	 */
	 virtual void step(system_t* system, int min_time_steps, int max_time_steps, double integration_step);

protected:
	
	/**
	 * @brief A randomly sampled state.
	 */
	double* sample_state;

	/**
	 * @brief A randomly sampled control.
	 */
	double* sample_control;

	/**
	 * @brief A resulting duration of a propagation step.
	 */
	double duration;

	/**
	 * @brief Storage used to query the nearest neighbor structure.
	 */
	tree_node_t* metric_query;

	/**
	 * @brief The result of a query in the nearest neighbor structure.
	 */
	sst_node_t* nearest;

	/**
	 * @brief The best goal node found so far.
	 */
	sst_node_t* best_goal;

	/**
	 * @brief A temporary storage for quering for close witnesses.
	 */
	sample_node_t* witness_sample;

	/**
	 * @brief A set of nodes used to get sets of nodes from the nearest neighbor structure.
	 */
	proximity_node_t** close_nodes;

	/**
	 * @brief A set of distances used to get sets of nodes from the nearest neighbor structure.
	 */
	double* distances;

	/**
	 * @brief Perform the random sampling step of SST.
	 * @details Perform the random sampling step of SST.
	 */
	void random_sample();

	/**
	 * @brief Finds a node to propagate from.
	 * @details Finds a node to propagate from. It does this through a procedure called BestNear w
	 * which examines a neighborhood around a randomly sampled point and returns the lowest cost one.
	 */
	void nearest_vertex();

	/**
	 * @brief If propagation was successful, add the new state to the tree.
	 * @details If propagation was successful, add the new state to the tree.
	 */
	void add_to_tree();

	/**
	 * @brief Add a state into the nearest neighbor structure for retrieval in later iterations.
	 * @details Add a state into the nearest neighbor structure for retrieval in later iterations.
	 * 
	 * @param node The node to add.
	 */
	void add_point_to_metric(tree_node_t* node);

	/**
	 * @brief Add a sample into the nearest neighbor structure for retrieval in later iterations.
	 * @details Add a sample into the nearest neighbor structure for retrieval in later iterations.
	 * 
	 * @param node The sample to add.
	 */
	void add_point_to_samples(tree_node_t* node);

	/**
	 * @brief Check if the currently created state is close to a witness.
	 * @details Check if the currently created state is close to a witness.
	 */
	void check_for_witness();

	/**
	 * @brief Removes a node from the nearest neighbor structure.
	 * @details Removes a node from the nearest neighbor structure.
	 * 
	 * @param node The node to remove.
	 */
	void remove_point_from_metric(tree_node_t* node);

	/**
	 * @brief Checks if this node has any children.
	 * @details Checks if this node has any children.
	 * 
	 * @param node The node to examine.
	 * @return True if no children, false if at least one child.
	 */
	bool is_leaf(tree_node_t* node);

	/**
	 * @brief Checks if this node is on the solution path.
	 * @details Checks if this node is on the solution path.
	 * 
	 * @param v The node to check
	 * @return True if on the solution path, false if not.
	 */
	bool is_best_goal(tree_node_t* v);

	/**
	 * @brief Removes a leaf node from the tree.
	 * @details Removes a leaf node from the tree.
	 * 
	 * @param node The node to remove.
	 */
	void remove_leaf(tree_node_t* node);

	void branch_and_bound(sst_node_t* node);

	/**
	 * The nearest neighbor structure for witness samples.
	 */
	graph_nearest_neighbors_t* samples;

	double sst_delta_near;
	double sst_delta_drain;


};

#endif