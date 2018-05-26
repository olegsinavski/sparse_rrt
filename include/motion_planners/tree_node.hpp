/**
 * @file tree_node.hpp
 * 
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2014, Rutgers the State University of New Jersey, New Brunswick  
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 * 
 * Authors: Zakary Littlefield, Kostas Bekris 
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
    tree_edge_t(const double* const a_control, double a_duration)
        : control(a_control)
        , duration(a_duration)
    { }

	double get_duration() const {
	    return duration;
	}

	const double* get_control() const {
	    return control;
	}

	void dealloc_control() {
        delete[] control;
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
	const double* control;
};

/**
 * @brief A point in the state space
 * @details A point in the state space that has attributes to be in the nearest neighbour structure
 */
class state_point_t
{
public:
	state_point_t(double* a_point)
	    : point(a_point)
	    , prox_node(NULL)
	{
	}

    double* get_point() const {
        return this->point;
    }

    void dealloc_point() {
        delete[] point;
        point = NULL;
    }

    void set_proximity_node(proximity_node_t* proximity_node) {
        this->prox_node = proximity_node;
    }

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
 * @brief A node of the tree.
 * @details A node of the tree.
 */
class tree_node_t: public state_point_t
{
public:
	tree_node_t(double* a_point, tree_edge_t* a_parent_edge, double a_cost)
	    : state_point_t(a_point)
	    , parent_edge(a_parent_edge)
	    , cost(a_cost)
	{
	}

    const std::list<tree_node_t*>& get_children() const {
        return this->children;
    }

    void add_child(tree_node_t* node) {
        this->children.insert(this->children.begin(), node);
    }

    void remove_child(tree_node_t* node) {
        this->children.remove(node);
    }

    bool is_leaf() const {
        return this->children.size()==0;
    }

    tree_edge_t* get_parent_edge() const {
        return this->parent_edge;
    }

    double get_cost() const {
        return this->cost;
    }

private:
     /**
     * @brief Children vertices
     */
    std::list<tree_node_t*> children;
    /**
    * @brief Parent edge
    */
    tree_edge_t* parent_edge;

     /**
     * @brief The path cost to this node.
     */
    double cost;
};

#endif