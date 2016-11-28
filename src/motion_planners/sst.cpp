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
	//init internal variables
	sample_state = this->alloc_state_point();
	sample_control = this->alloc_control_point();
	metric_query = new sst_node_t();
	metric_query->point = this->alloc_state_point();

    close_nodes = (proximity_node_t**)malloc(MAX_KK * sizeof (proximity_node_t*));
    distances = (double*)malloc(MAX_KK * sizeof (double));

	//initialize the metrics
	metric = new graph_nearest_neighbors_t();
	metric->set_system(system);
	//create the root of the tree
	root = new sst_node_t();
	root->point = this->alloc_state_point();
	this->copy_state_point(root->point,start_state);
	add_point_to_metric(root);
	number_of_nodes++;

	samples = new graph_nearest_neighbors_t();
	samples->set_system(system);
	witness_sample = new sample_node_t();
	witness_sample->point = this->alloc_state_point();
	this->copy_state_point(witness_sample->point,start_state);
	add_point_to_samples(witness_sample);

	witness_sample->rep = (sst_node_t*)root;

}
void sst_t::get_solution(std::vector<std::pair<double*,double> >& controls)
{
	last_solution_path.clear();
	if(best_goal==NULL)
		return;
	nearest = best_goal;
	
	//now nearest should be the closest node to the goal state
	std::deque<tree_node_t*> path;
	while(nearest->parent!=NULL)
	{
		path.push_front(nearest);
		nearest = (sst_node_t*)nearest->parent;
	}
	last_solution_path.push_back(root);
	for(unsigned i=0;i<path.size();i++)
	{
		last_solution_path.push_back(path[i]);
		controls.push_back(std::pair<double*,double>(NULL,0));
		controls.back().first = this->alloc_control_point();
		this->copy_control_point(controls.back().first,path[i]->parent_edge->control);
		controls.back().second = path[i]->parent_edge->duration;
	}
}
void sst_t::step(int min_time_steps, int max_time_steps, double integration_step)
{
	random_sample();
	nearest_vertex();
	if(propagate(min_time_steps, max_time_steps, integration_step))
	{
		add_to_tree();
	}
}

void sst_t::add_point_to_metric(tree_node_t* state)
{
	proximity_node_t* new_node = new proximity_node_t(state);
	state->prox_node = new_node;
	metric->add_node(new_node);
}

void sst_t::add_point_to_samples(tree_node_t* state)
{
	proximity_node_t* new_node = new proximity_node_t(state);
	state->prox_node = new_node;
	samples->add_node(new_node);
}


void sst_t::random_sample()
{
	this->random_state(sample_state);
	this->random_control(sample_control);
}
void sst_t::nearest_vertex()
{
	//performs the best near query
	this->copy_state_point(metric_query->point,sample_state);
	unsigned val = metric->find_delta_close_and_closest(metric_query,close_nodes, distances, this->sst_delta_near);

    double length = 999999999;
    for(unsigned i=0;i<val;i++)
    {
        tree_node_t* v = (tree_node_t*)(close_nodes[i]->get_state());
        double temp = v->cost ;
        if( temp < length)
        {
            length = temp;
            nearest = (sst_node_t*)v;
        }
    }
}
bool sst_t::propagate(int min_time_steps, int max_time_steps, double integration_step)
{
	return system->propagate(nearest->point,sample_control, min_time_steps, max_time_steps, sample_state, duration, integration_step);
}
void sst_t::add_to_tree()
{
	//check to see if a sample exists within the vicinity of the new node
	check_for_witness();

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
			this->copy_control_point(new_node->parent_edge->control,sample_control);
			new_node->parent_edge->duration = duration;
			//set this node's parent
			new_node->parent = nearest;
			new_node->cost = nearest->cost + duration;
			//set parent's child
			nearest->children.insert(nearest->children.begin(),new_node);
			number_of_nodes++;

	        if(best_goal==NULL && system->distance(new_node->point,goal_state)<goal_radius)
	        {
	        	best_goal = new_node;
	        	branch_and_bound((sst_node_t*)root);
	        }
	        else if(best_goal!=NULL && best_goal->cost > new_node->cost && system->distance(new_node->point,goal_state)<goal_radius)
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

void sst_t::check_for_witness()
{
	this->copy_state_point(metric_query->point,sample_state);
	double distance;
	witness_sample = (sample_node_t*)samples->find_closest(metric_query,&distance)->get_state();
	if(distance > this->sst_delta_drain)
	{
		//create a new sample
		witness_sample = new sample_node_t();
		witness_sample->point = this->alloc_state_point();
		this->copy_state_point(witness_sample->point,sample_state);
		add_point_to_samples(witness_sample);
	}
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
	metric->remove_node(old_node);
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

