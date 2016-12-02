/**
 * @file sst.cpp
 * 
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2014, Rutgers the State University of New Jersey, New Brunswick  
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 * 
 * Authors: Zakary Littlefield, Kostas Bekris 
 * 
 */

#include "motion_planners/sst.hpp"
#include "nearest_neighbors/graph_nearest_neighbors.hpp"

#include <iostream>
#include <deque>


void sst_t::setup_planning()
{
	best_goal = NULL;

	//initialize the metrics
	metric.set_distance(this->distance);
	//create the root of the tree
	root = new sst_node_t();
	root->point = this->alloc_state_point();
	this->copy_state_point(root->point,start_state);
	add_point_to_metric(root);
	number_of_nodes++;

	samples.set_distance(this->distance);

    sample_node_t* first_witness_sample = new sample_node_t();
    first_witness_sample->point = this->alloc_state_point();
	this->copy_state_point(first_witness_sample->point, start_state);
    first_witness_sample->rep = (sst_node_t*)root;

	add_point_to_samples(first_witness_sample);
}

void sst_t::get_solution(std::vector<std::vector<double>>& solution_path, std::vector<std::vector<double>>& controls, std::vector<double>& costs)
{
	if(best_goal==NULL)
		return;
	tree_node_t* nearest_path_node = best_goal;
	
	//now nearest_path_node should be the closest node to the goal state
	std::deque<tree_node_t*> path;
	while(nearest_path_node->parent!=NULL)
	{
		path.push_front(nearest_path_node);
        nearest_path_node = nearest_path_node->parent;
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
void sst_t::step(system_t* system, int min_time_steps, int max_time_steps, double integration_step)
{
    /*
     * Generate a random sample
     * Find the closest existing node
     * Generate random control
     * Propagate for random time with constant random control from the closest node
     * If resulting state is valid, add a resulting state into the tree and perform sst-specific graph manipulations
     */

    double* sample_state = this->alloc_state_point();
    double* sample_control = this->alloc_control_point();
	this->random_state(sample_state);
	this->random_control(sample_control);
    sst_node_t* nearest = nearest_vertex(sample_state);
	int num_steps = this->random_generator.uniform_int_random(min_time_steps, max_time_steps);
    double duration = num_steps*integration_step;
	if(system->propagate(nearest->point, sample_control, num_steps, sample_state, integration_step))
	{
		add_to_tree(sample_state, sample_control, nearest, duration);
	}
    delete sample_state;
    delete sample_control;
}

void sst_t::add_point_to_metric(tree_node_t* state)
{
	proximity_node_t* new_node = new proximity_node_t(state);
	state->prox_node = new_node;
	metric.add_node(new_node);
}

void sst_t::add_point_to_samples(tree_node_t* state)
{
	proximity_node_t* new_node = new proximity_node_t(state);
	state->prox_node = new_node;
	samples.add_node(new_node);
}

sst_node_t* sst_t::nearest_vertex(const double* sample_state)
{
	//performs the best near query
    std::vector<proximity_node_t*> close_nodes = metric.find_delta_close_and_closest(sample_state, this->sst_delta_near);

    double length = 999999999;
    sst_node_t* nearest = nullptr;
    for(unsigned i=0;i<close_nodes.size();i++)
    {
        tree_node_t* v = (tree_node_t*)(close_nodes[i]->get_state());
        double temp = v->cost ;
        if( temp < length)
        {
            length = temp;
            nearest = (sst_node_t*)v;
        }
    }
    assert (nearest != nullptr);
    return nearest;
}

void sst_t::add_to_tree(const double* sample_state, const double* sample_control, sst_node_t* nearest, double duration)
{
	//check to see if a sample exists within the vicinity of the new node
    sample_node_t* witness_sample = find_witness(sample_state);

	if(witness_sample->rep==NULL || witness_sample->rep->cost > nearest->cost + duration)
	{
		if(best_goal==NULL || nearest->cost + duration <= best_goal->cost)
		{
			//create a new tree node
			sst_node_t* new_node = new sst_node_t();
			new_node->point = this->alloc_state_point();
			this->copy_state_point(new_node->point,sample_state);
			//create the link to the parent node
			new_node->parent_edge = new tree_edge_t();
			new_node->parent_edge->control = this->alloc_control_point();
			this->copy_control_point(new_node->parent_edge->control, sample_control);
			new_node->parent_edge->duration = duration;
			//set this node's parent
			new_node->parent = nearest;
			new_node->cost = nearest->cost + duration;
			//set parent's child
			nearest->children.insert(nearest->children.begin(),new_node);
			number_of_nodes++;

	        if(best_goal==NULL && this->distance(new_node->point,goal_state)<goal_radius)
	        {
	        	best_goal = new_node;
	        	branch_and_bound((sst_node_t*)root);
	        }
	        else if(best_goal!=NULL && best_goal->cost > new_node->cost && this->distance(new_node->point,goal_state)<goal_radius)
	        {
	        	best_goal = new_node;
	        	branch_and_bound((sst_node_t*)root);
	        }


			if(witness_sample->rep!=NULL)
			{
				//optimization for sparsity
				if(!(witness_sample->rep->inactive))
				{
					remove_point_from_metric(witness_sample->rep);
					witness_sample->rep->inactive = true;
				}

	            sst_node_t* iter = witness_sample->rep;
	            while( is_leaf(iter) && iter->inactive && !is_best_goal(iter))
	            {
	                sst_node_t* next = (sst_node_t*)iter->parent;
	                remove_leaf(iter);
	                iter = next;
	            } 

			}
			witness_sample->rep = new_node;
			new_node->witness = witness_sample;
			add_point_to_metric(new_node);
		}
	}	

}

sample_node_t* sst_t::find_witness(const double* sample_state)
{
	double distance;
    sample_node_t* witness_sample = samples.find_closest(sample_state, &distance)->get_state();
	if(distance > this->sst_delta_drain)
	{
		//create a new sample
		witness_sample = new sample_node_t();
		witness_sample->point = this->alloc_state_point();
		this->copy_state_point(witness_sample->point, sample_state);
		add_point_to_samples(witness_sample);
	}
    return witness_sample;
}

void sst_t::branch_and_bound(sst_node_t* node)
{
    std::list<tree_node_t*> children = node->children;
    for (std::list<tree_node_t*>::iterator iter = children.begin(); iter != children.end(); ++iter)
    {
    	branch_and_bound((sst_node_t*)(*iter));
    }
    if(is_leaf(node) && node->cost > best_goal->cost)
    {
    	if(!node->inactive)
    	{
	    	node->witness->rep = NULL;
	    	remove_point_from_metric(node);
	    }
    	remove_leaf(node);
    }
}

void sst_t::remove_point_from_metric(tree_node_t* node)
{
	proximity_node_t* old_node = node->prox_node;
    node->prox_node = nullptr;
	metric.remove_node(old_node);
	delete old_node;
}

bool sst_t::is_leaf(tree_node_t* node)
{
	return node->children.size()==0;
}

void sst_t::remove_leaf(tree_node_t* node)
{
	if(node->parent != NULL)
	{
		tree_edge_t* edge = node->parent_edge;
		node->parent->children.remove(node);
		number_of_nodes--;
		delete edge->control;
		delete node->point;
		delete node;
	}
}

bool sst_t::is_best_goal(tree_node_t* v)
{
	if(best_goal==NULL)
		return false;
    tree_node_t* new_v = best_goal;

    while(new_v->parent!=NULL)
    {
        if(new_v == v)
            return true;

        new_v = new_v->parent;
    }
    return false;

}

