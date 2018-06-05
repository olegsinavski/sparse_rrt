/**
 * @file tree_node.hpp
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

#ifndef SPARSE_TREE_NODE_HPP
#define SPARSE_TREE_NODE_HPP

#include <list>
#include <cstddef>

class proximity_node_t;

/**
 * @brief An edge of the tree.
 * @details An edge of the tree.
 */
class tree_edge_t
{
public:
	/**
	 * @brief Edge Constructor
	 * @details Edge Constructor
	 *
	 * @param a_control A point in the control space
	 * @param control_dimension Dimensionality of the control space
	 * @param a_duration The time duration of the edge
	 */
    tree_edge_t(const double* a_control, unsigned int control_dimension, double a_duration)
        : duration(a_duration)
        , control(new double[control_dimension])
    {
        if (a_control) {
            std::copy(a_control, a_control + control_dimension, this->control);
        }
    }

	/**
	 * @brief Edge move constructor
	 * @details Edge move constructor
	 *
	 * @param other Edge r-value
	 */
    tree_edge_t(tree_edge_t&& other)
        : duration(other.duration)
        , control(other.control)
    {
        other.control = nullptr;
        other.duration = -1;
    }

	/**
	 * @brief Return duration of the edge
	 * @details Return duration of the edge
	 *
	 * @return Duration of the control
	 */
	double get_duration() const {
	    return duration;
	}

	/**
	 * @brief Return control point attached to this edge
	 * @details Return control point attached to this edge
	 *
	 * @return control point attached to this edge
	 */
	const double* get_control() const {
	    return control;
	}

	~tree_edge_t() {
        if (control != NULL) {
            delete[] control;
        }
        control = NULL;
    }

private:

    /**
     * @brief The duration to execute control.
     */
	double duration;
	 /**
     * @brief The control for this edge.
     */
	double* control;
};

/**
 * @brief A point in the state space
 * @details A point in the state space that has attributes to be in the nearest neighbour structure
 */
class state_point_t
{
public:
	/**
	 * @brief State point constructor
	 * @details State point Constructor
	 *
	 * @param a_point A point in the state space
	 * @param state_dimension Dimensionality of the state space
	 */
	state_point_t(const double* a_point, unsigned int state_dimension)
	    : point(new double[state_dimension])
	    , prox_node(NULL)
	{
	    if (a_point) {
            std::copy(a_point, a_point + state_dimension, this->point);
        }
	}

	state_point_t(const state_point_t&) = delete;

	virtual ~state_point_t() {
	    if (point) {
	        delete[] point;
            point = NULL;
	    }
	}

	/**
	 * @brief Get state space point
	 * @details Get state space point
	 *
	 * @return state space point
	 */
    const double* get_point() const {
        return this->point;
    }

	/**
	 * @brief Set proximity node from NN structure
	 * @details Set proximity node from NN structure
	 *
	 * @param proximity_node NN structure node
	 */
    void set_proximity_node(proximity_node_t* proximity_node) {
        this->prox_node = proximity_node;
    }

	/**
	 * @brief Return NN node attached to this node
	 * @details Return NN node attached to this node
	 *
	 * @return NN node attached to this node
	 */
    const proximity_node_t* get_proximity_node() const {
        return this->prox_node;
    }
private:
     /**
     * @brief The state represented by this node.
     */
    double* point;
    /**
     * @brief A pointer to the node in the nearest neighbor structure for easy removal.
     */
    proximity_node_t* prox_node;
};


/**
 * @brief A node of the tree
 * @details A node of the tree (state space point, parent and parent edge)
 */
class tree_node_t: public state_point_t
{
public:
	/**
	 * @brief Tree point constructor
	 * @details State point Constructor
	 *
	 * @param a_point A point in the state space
	 * @param state_dimension Dimensionality of the state space
	 * @param a_parent_edge Edge to the parent node
	 * @param cost The path cost to this node.
	 */
	tree_node_t(const double* a_point, unsigned int state_dimension, tree_edge_t&& a_parent_edge, double a_cost)
	    : state_point_t(a_point, state_dimension)
	    , parent_edge(std::move(a_parent_edge))
	    , cost(a_cost)
	{
	}

    virtual ~tree_node_t() {
        for (auto child: this->children) {
            delete child;
        }
	}

	/**
	 * @brief Return children of this node
	 * @details Return children of this node
	 *
	 * @return children of this node
	 */
    const std::list<tree_node_t*>& get_children() const {
        return this->children;
    }

	/**
	 * @brief Add new child to this node
	 * @details Add new child to this node
	 *
	 * @param node Child tree_node_t node
	 * @return child node
	 */
    tree_node_t* add_child(tree_node_t* node) {
        this->children.insert(this->children.begin(), node);
        return node;
    }

	/**
	 * @brief Remove a child from this node
	 * @details Remove a child from this node
	 *
	 * @param node Child tree_node_t node
	 */
    void remove_child(tree_node_t* node) {
        this->children.remove(node);
    }

	/**
	 * @brief Check if the node doesn't have children
	 * @details Check if the node doesn't have children
	 *
	 * @return whether the node is leaf (doesn't have children)
	 */
    bool is_leaf() const {
        return this->children.size()==0;
    }

	/**
	 * @brief Return an edge to the parent node
	 * @details Return an edge to the parent node
	 *
	 * @return edge to the parent node
	 */
    const tree_edge_t& get_parent_edge() const {
        return this->parent_edge;
    }

	/**
	 * @brief Return cost of the parent edge
	 * @details Return cost of the parent edge
	 *
	 * @return cost of the parent edge
	 */
    double get_cost() const {
        return this->cost;
    }

private:
    /**
    * @brief Parent edge
    */
    tree_edge_t parent_edge;

     /**
     * @brief The path cost to this node.
     */
    double cost;
     /**
     * @brief Children vertices
     */
    std::list<tree_node_t*> children;
};

#endif