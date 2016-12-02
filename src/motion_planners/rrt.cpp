/**
 * @file rrt.cpp
 * 
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2014, Rutgers the State University of New Jersey, New Brunswick  
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 * 
 * Authors: Zakary Littlefield, Kostas Bekris 
 * 
 */

#include "motion_planners/rrt.hpp"
#include "nearest_neighbors/graph_nearest_neighbors.hpp"

#include <iostream>
#include <deque>

void rrt_t::setup_planning()
{
    //init internal variables
    sample_state = this->alloc_state_point();
    sample_control = this->alloc_control_point();

    //initialize the metric
    metric = new graph_nearest_neighbors_t();
    metric->set_distance(this->distance);
    //create the root of the tree
    root = new tree_node_t();
    number_of_nodes++;
    root->point = this->alloc_state_point();
    this->copy_state_point(root->point,start_state);
    //add root to nearest neighbor structure
    add_point_to_metric(root);

}
void rrt_t::get_solution(std::vector<std::vector<double>>& solution_path, std::vector<std::vector<double>>& controls, std::vector<double>& costs)
{
    this->copy_state_point(sample_state,goal_state);
    std::vector<proximity_node_t*> close_nodes = metric->find_delta_close_and_closest(sample_state, goal_radius);

    double length = 999999999;
    for(unsigned i=0;i<close_nodes.size();i++)
    {
        tree_node_t* v = (tree_node_t*)(close_nodes[i]->get_state());
        double temp = v->cost ;
        if( temp < length)
        {
            length = temp;
            nearest = v;
        }
    }
    //now nearest should be the closest node to the goal state
    if(this->distance(goal_state,nearest->point) < goal_radius)
    {
        std::deque<tree_node_t*> path;
        while(nearest->parent!=NULL)
        {
            path.push_front(nearest);
            nearest = nearest->parent;
        }

        std::vector<double> root_state;
        for (unsigned c=0; c<this->state_dimension; c++) {
            root_state.push_back(root->point[c]);
        }
        solution_path.push_back(root_state);

        for(unsigned i=0;i<path.size();i++)
        {
            std::vector<double> current_state;
            for (unsigned c=0; c<this->state_dimension; c++) {
                current_state.push_back(path[i]->point[c]);
            }
            solution_path.push_back(current_state);

            std::vector<double> current_control;
            for (unsigned c=0; c<this->control_dimension; c++) {
                current_control.push_back(path[i]->parent_edge->control[c]);
            }
            controls.push_back(current_control);
            costs.push_back(path[i]->parent_edge->duration);
        }
    }
}
void rrt_t::step(system_t* system, int min_time_steps, int max_time_steps, double integration_step)
{
    this->random_state(sample_state);
    this->random_control(sample_control);

    nearest_vertex();
    int num_steps = this->random_generator.uniform_int_random(min_time_steps, max_time_steps);
    this->duration = num_steps*integration_step;
    if(system->propagate(nearest->point, sample_control, num_steps, sample_state, integration_step))
    {
        add_to_tree();
    }
}

void rrt_t::add_point_to_metric(tree_node_t* state)
{
    proximity_node_t* new_node = new proximity_node_t(state);
    state->prox_node = new_node;
    metric->add_node(new_node);
}

void rrt_t::nearest_vertex()
{
    double distance;
    nearest = (tree_node_t*)metric->find_closest(sample_state, &distance)->get_state();
}

void rrt_t::add_to_tree()
{
    //create a new tree node
    tree_node_t* new_node = new tree_node_t();
    new_node->point = this->alloc_state_point();
    this->copy_state_point(new_node->point,sample_state);
    //create the link to the parent node
    new_node->parent_edge = new tree_edge_t();
    new_node->parent_edge->control = this->alloc_control_point();
    this->copy_control_point(new_node->parent_edge->control,sample_control);
    new_node->parent_edge->duration = duration;
    //set this node's parent
    new_node->parent = nearest;
    new_node->cost = nearest->cost + duration;
    //set parent's child
    nearest->children.insert(nearest->children.begin(),new_node);
    add_point_to_metric(new_node);
    number_of_nodes++;
}

