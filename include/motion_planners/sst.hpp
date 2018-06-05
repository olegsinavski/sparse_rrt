/**
 * @file sst.hpp
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
	/**
	 * @brief SST node constructor
	 * @details SST node constructor
	 *
	 * @param point State space point
	 * @param state_dimension Dimensionality of the state space
	 * @param a_parent Parent node in the planning graph
	 * @param a_parent_edge An edge between the parent and this node
	 * @param a_cost Cost of the edge
	 */
	sst_node_t(const double* point, unsigned int state_dimension, sst_node_t* a_parent, tree_edge_t&& a_parent_edge, double a_cost);
	~sst_node_t();

    /**
	 * @brief Return whether the node is active
	 * @details Return whether the node is active
	 *
	 * @return witness node pointer
	 */
	bool is_active() const {
	    return active;
	}

    /**
	 * @brief Inactivate the node
	 * @details Inactivate the node
	 *
	 */
	void make_inactive() {
	    this->active = false;
	}

    /**
	 * @brief Set a witness node for this point
	 * @details Set a witness node for this point
	 *
	 * @param a_witness Witness node pointer
	 */
	void set_witness(sample_node_t* a_witness) {
	    this->witness = a_witness;
	}

    /**
	 * @brief Return the witness node
	 * @details Return the witness node
	 *
	 * @return witness node pointer
	 */
    sample_node_t* get_witness() const {
	    return this->witness;
	}

    /**
	 * @brief Return the parent node
	 * @details Return the parent node
	 *
	 * @return parent node pointer
	 */
    sst_node_t* get_parent() const {
        return this->parent;
    }

private:
    /**
     * @brief Parent node.
     */
    sst_node_t* parent;

	/**
	 * A flag for inclusion in the metric.
	 */
	bool active;
    sample_node_t* witness;
};

/**
 * @brief A special storage node for witness nodes in SST.
 * @details A special storage node for witness nodes in SST.
 * 
 */
class sample_node_t : public state_point_t
{
public:
	/**
	 * @brief Witness node constructor
	 * @details Witness node constructor
	 *
	 * @param representative SST pointer node
	 * @param a_point A point in the state space
	 * @param state_dimension Dimensionality of the state space
	 */
	sample_node_t(sst_node_t* const representative,
	              const double* a_point, unsigned int state_dimension);
	~sample_node_t();

    /**
	 * @brief Set a representative node for this point
	 * @details Set a representative node for this point
	 *
	 * @param a_witness Witness node pointer
	 */
    void set_representative(sst_node_t* const representative) {
        this->rep = representative;
    }

    /**
	 * @brief Return the node this witness represents
	 * @details Return the node this witness represents
	 *
	 * @return node this witness represents
	 */
    sst_node_t* get_representative() const {
        return this->rep;
    }
private:
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
	 * @brief SST planner Constructor
	 * @details SST planner Constructor
	 *
	 * @param in_start The start state.
	 * @param in_goal The goal state
	 * @param in_radius The radial size of the goal region centered at in_goal.
	 * @param a_state_bounds A vector with boundaries of the state space (min and max)
	 * @param a_control_bounds A vector with boundaries of the control space (min and max)
	 * @param distance_function Function that returns distance between two state space points
	 * @param random_seed The seed for the random generator
	 * @param delta_near Near distance threshold for SST
	 * @param delta_drain Drain distance threshold for SST
	 */
	sst_t(const double* in_start, const double* in_goal,
	      double in_radius,
	      const std::vector<std::pair<double, double> >& a_state_bounds,
		  const std::vector<std::pair<double, double> >& a_control_bounds,
		  std::function<double(const double*, const double*, unsigned int)> distance_function,
		  unsigned int random_seed,
		  double delta_near, double delta_drain);
	virtual ~sst_t();

	/**
	 * @copydoc planner_t::get_solution(std::vector<std::pair<double*,double> >&)
	 */
	virtual void get_solution(std::vector<std::vector<double>>& solution_path, std::vector<std::vector<double>>& controls, std::vector<double>& costs);
	
	/**
	 * @copydoc planner_t::step()
	 */
	 virtual void step(system_interface* system, int min_time_steps, int max_time_steps, double integration_step);

protected:

    /**
     * @brief The nearest neighbor data structure.
     */
    graph_nearest_neighbors_t metric;

	/**
	 * @brief The best goal node found so far.
	 */
	sst_node_t* best_goal;

	/**
	 * @brief Finds a node to propagate from.
	 * @details Finds a node to propagate from. It does this through a procedure called BestNear w
	 * which examines a neighborhood around a randomly sampled point and returns the lowest cost one.
	 */
	sst_node_t* nearest_vertex(const double* sample_state);

	/**
	 * @brief If propagation was successful, add the new state to the tree.
	 * @details If propagation was successful, add the new state to the tree.
	 */
	void add_to_tree(const double* sample_state, const double* sample_control, sst_node_t* nearest, double duration);

	/**
	 * @brief Check if the currently created state is close to a witness.
	 * @details Check if the currently created state is close to a witness.
	 */
	sample_node_t* find_witness(const double* sample_state);

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
	void remove_leaf(sst_node_t* node);

	/**
	 * @brief Branch out and prune planning tree
	 * @details Branch out and prune planning tree
	 *
	 * @param node The node from which to branch
	 */
	void branch_and_bound(sst_node_t* node);

	/**
	 * The nearest neighbor structure for witness samples.
	 */
	graph_nearest_neighbors_t samples;

	/**
	 * @brief Near distance threshold for SST
	 */
	double sst_delta_near;

	/**
	 * @brief Drain distance threshold for SST
	 */
	double sst_delta_drain;

	/**
	 * @brief Container for witness nodes (to avoid memory leaks)
	 */
    std::vector<sample_node_t*> witness_nodes;
};

#endif