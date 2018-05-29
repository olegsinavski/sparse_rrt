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

void rrt_t::setup_rrt_planning()
{
    //init internal variables
    sample_state = new double[this->state_dimension];
    sample_control = new double[this->control_dimension];

    //initialize the metric
    metric = new graph_nearest_neighbors_t();
    metric->set_distance(this->distance);

    this->root = new rrt_node_t(start_state, this->state_dimension, NULL, tree_edge_t(NULL, 0, -1.), 0.);
    number_of_nodes++;

    //add root to nearest neighbor structure
    add_point_to_metric(root);

}
void rrt_t::get_solution(std::vector<std::vector<double>>& solution_path, std::vector<std::vector<double>>& controls, std::vector<double>& costs)
{
    std::copy(goal_state, goal_state + this->state_dimension, sample_state);
    std::vector<proximity_node_t*> close_nodes = metric->find_delta_close_and_closest(sample_state, goal_radius);

    double length = std::numeric_limits<double>::max();;
    for(unsigned i=0;i<close_nodes.size();i++)
    {
        rrt_node_t* v = (rrt_node_t*)(close_nodes[i]->get_state());
        double temp = v->get_cost() ;
        if( temp < length)
        {
            length = temp;
            nearest = v;
        }
    }
    //now nearest should be the closest node to the goal state
    if(this->distance(goal_state,nearest->get_point()) < goal_radius)
    {
        std::deque<const rrt_node_t*> path;
        while(nearest->get_parent()!=NULL)
        {
            path.push_front(nearest);
            nearest = nearest->get_parent();
        }

        std::vector<double> root_state;
        for (unsigned c=0; c<this->state_dimension; c++) {
            root_state.push_back(root->get_point()[c]);
        }
        solution_path.push_back(root_state);

        for(unsigned i=0;i<path.size();i++)
        {
            std::vector<double> current_state;
            for (unsigned c=0; c<this->state_dimension; c++) {
                current_state.push_back(path[i]->get_point()[c]);
            }
            solution_path.push_back(current_state);

            std::vector<double> current_control;
            for (unsigned c=0; c<this->control_dimension; c++) {
                current_control.push_back(path[i]->get_parent_edge().get_control()[c]);
            }
            controls.push_back(current_control);
            costs.push_back(path[i]->get_parent_edge().get_duration());
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
    if(system->propagate(nearest->get_point(), sample_control, num_steps, sample_state, integration_step))
    {
        add_to_tree();
    }
}

void rrt_t::add_point_to_metric(tree_node_t* state)
{
    metric->add_node(state);
}

void rrt_t::nearest_vertex()
{
    double distance;
    nearest = (rrt_node_t*)(metric->find_closest(sample_state, &distance)->get_state());
}

void rrt_t::add_to_tree()
{
    //create a new tree node
    rrt_node_t* new_node = new rrt_node_t(
        sample_state, this->state_dimension, nearest,
        tree_edge_t(sample_control, this->control_dimension, duration),
        nearest->get_cost() + duration);

    //set parent's child
    nearest->add_child(new_node);
    add_point_to_metric(new_node);
    number_of_nodes++;
}

