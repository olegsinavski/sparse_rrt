/**
 * @file rrt.hpp
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

#ifndef SPARSE_PLANNER_RRT_HPP
#define SPARSE_PLANNER_RRT_HPP

#include "systems/system.hpp"
#include "motion_planners/planner.hpp"


/**
 * @brief A special storage node for RRT.
 * @details A special storage node for RRT.
 */
class rrt_node_t : public tree_node_t
{
public:
	/**
	 * @brief RRT node constructor
	 * @details RRT node constructor
	 *
	 * @param point State space point
	 * @param state_dimension Dimensionality of the state space
	 * @param a_parent Parent node in the planning graph
	 * @param a_parent_edge An edge between the parent and this node
	 * @param a_cost Cost of the edge
	 */
	rrt_node_t(double* point, unsigned int state_dimension, rrt_node_t* a_parent, tree_edge_t&& a_parent_edge, double a_cost);

	~rrt_node_t();

    /**
	 * @brief Return the parent node
	 * @details Return the parent node
	 *
	 * @return parent node pointer
	 */
    rrt_node_t* get_parent() const {
        return this->parent;
    }

private:
    /**
     * @brief Parent node.
     */
    rrt_node_t* parent;
};

/**
 * @brief The motion planning algorithm RRT (Rapidly-exploring Random Tree)
 * @details The motion planning algorithm RRT (Rapidly-exploring Random Tree)
 */
class rrt_t : public planner_t
{
public:
	/**
	 * @copydoc planner_t::planner_t(system_t*)
	 */
	rrt_t(const double* in_start, const double* in_goal,
	      double in_radius,
	      const std::vector<std::pair<double, double> >& a_state_bounds,
		  const std::vector<std::pair<double, double> >& a_control_bounds,
		  std::function<double(const double*, const double*, unsigned int)> a_distance_function,
          unsigned int random_seed)
			: planner_t(in_start, in_goal, in_radius,
			            a_state_bounds, a_control_bounds, a_distance_function, random_seed)
	{
        //initialize the metric
        unsigned int state_dimensions = this->get_state_dimension();
        metric.set_distance(
            [state_dimensions, a_distance_function](const double* s0, const double* s1) {
                return a_distance_function(s0, s1, state_dimensions);
            }
        );

        this->root = new rrt_node_t(start_state, this->state_dimension, NULL, tree_edge_t(NULL, 0, -1.), 0.);
        number_of_nodes++;

        //add root to nearest neighbor structure
        metric.add_node(root);
	}
	virtual ~rrt_t(){
        delete this->root;
        this->root = nullptr;
	}

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
	 * @brief The result of a query in the nearest neighbor structure.
	 */
	rrt_node_t* nearest;

	/**
	 * @brief Find the nearest node to the randomly sampled state.
	 * @details Find the nearest node to the randomly sampled state.
	 */
	rrt_node_t* nearest_vertex(const double* state) const;

};

#endif