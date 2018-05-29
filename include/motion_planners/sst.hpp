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
	sst_node_t(const double* point, unsigned int state_dimension, sst_node_t* a_parent, tree_edge_t&& a_parent_edge, double a_cost);
	~sst_node_t();

	bool is_active() const {
	    return active;
	}

	void make_inactive() {
	    this->active = false;
	}

	void set_witness(sample_node_t* a_witness) {
	    this->witness = a_witness;
	}

    sample_node_t* get_witness() const {
	    return this->witness;
	}

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
	sample_node_t(sst_node_t* const representative,
	              const double* a_point, unsigned int state_dimension)
	    : state_point_t(a_point, state_dimension)
	    , rep(representative)
	{ }

    void set_representative(sst_node_t* const representative) {
        this->rep = representative;
    }

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
	 * @copydoc planner_t::planner_t()
	 */
	sst_t(const double* in_start, const double* in_goal,
	      double in_radius,
	      const std::vector<std::pair<double, double> >& a_state_bounds,
		  const std::vector<std::pair<double, double> >& a_control_bounds,
		  std::function<double(const double*, const double*)> distance_function,
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
	 virtual void step(system_t* system, int min_time_steps, int max_time_steps, double integration_step);

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

	void branch_and_bound(sst_node_t* node);

	/**
	 * The nearest neighbor structure for witness samples.
	 */
	graph_nearest_neighbors_t samples;

	double sst_delta_near;
	double sst_delta_drain;

};

#endif