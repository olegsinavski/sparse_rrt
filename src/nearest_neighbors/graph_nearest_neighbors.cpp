/**
 * @file graph_nearest_neighbor.cpp
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

#include <assert.h>
#include <limits>
#include <cmath>

#include "nearest_neighbors/graph_nearest_neighbors.hpp"
#include "motion_planners/tree_node.hpp"

proximity_node_t::proximity_node_t( const state_point_t* st )
    : state(st)
{
}


const state_point_t* proximity_node_t::get_state( ) const
{
    return state;
}

int proximity_node_t::get_index() const
{
    return index;
}

void proximity_node_t::set_index( int indx )
{
    index = indx;
}

const std::vector<unsigned int>& proximity_node_t::get_neighbors() const
{
    return neighbors;
}

void proximity_node_t::add_neighbor( unsigned int nd )
{
    neighbors.push_back(nd);
}

void proximity_node_t::delete_neighbor( unsigned int nd )
{
    unsigned int index;
    for( index=0; index<neighbors.size(); index++ )
    {
        if( neighbors[index] == nd )
            break;
    }
    assert( index < neighbors.size() );

    for( unsigned int i=index; i<neighbors.size()-1; i++ ) {
        neighbors[i] = neighbors[i+1];
    }
	neighbors.pop_back();
}

void proximity_node_t::replace_neighbor( unsigned prev, int new_index )
{
    unsigned int index;
    for( index=0; index<neighbors.size(); index++ )
    {
        if( neighbors[index] == prev )
            break;
    }
    assert( index < neighbors.size() );

    neighbors[index] = new_index;
}


////////////////////////////////////////
////// SORTING
///////////////////////////////////////

void sort( proximity_node_t** close_nodes, double* distances, int low, int high )
{ 
    if( low < high )
    {
        int left, right;
        double pivot_distance = distances[low];
        proximity_node_t* pivot_node = close_nodes[low];
        left = low;
        right = high;
        while( left < right )
        {
            while( left <= high && distances[left] <= pivot_distance )
            left++;
            while( distances[right] > pivot_distance )
            right--;
            if( left < right )
            {
            double temp = distances[left];
            distances[left] = distances[right];
            distances[right] = temp;

            proximity_node_t* temp_node = close_nodes[left];
            close_nodes[left] = close_nodes[right];
            close_nodes[right] = temp_node;
            }
        }
        distances[low] = distances[right];
        distances[right] = pivot_distance;

        close_nodes[low] = close_nodes[right];
        close_nodes[right] = pivot_node;

        sort( close_nodes, distances, low, right-1 );
        sort( close_nodes, distances, right+1, high );
    }
}

unsigned int resort( proximity_node_t** close_nodes, double* distances, unsigned int index )
{
    while( index > 0 && distances[ index ] < distances[ index-1 ] )
    {
        double temp = distances[index];
        distances[index] = distances[index-1];
        distances[index-1] = temp;
        proximity_node_t* temp_node = close_nodes[index];
        close_nodes[index] = close_nodes[index-1];
        close_nodes[index-1] = temp_node;
        index--;
    }
    return index;
}


graph_nearest_neighbors_t::graph_nearest_neighbors_t()
    : nodes()
    , second_nodes(MAX_KK, nullptr)
    , second_distances(MAX_KK, 0.)
    , random_generator(0)
{
    nodes.reserve(INIT_NODE_SIZE);
}

graph_nearest_neighbors_t::~graph_nearest_neighbors_t()
{
    for (auto n: nodes) {
        delete n;
    }
    nodes.clear();
}

double graph_nearest_neighbors_t::average_valence() const
{
    double all_neighs = 0;
    for( unsigned int i=0; i<nodes.size(); i++ )
    {
        auto neighs = nodes[i]->get_neighbors();
        all_neighs += neighs.size();
    }
    all_neighs /= (double)nodes.size();
    return all_neighs;
}

void graph_nearest_neighbors_t::add_node( state_point_t* state )
{
    proximity_node_t* graph_node = new proximity_node_t(state);
	state->set_proximity_node(graph_node);

    int k = percolation_threshold();

    unsigned int new_k = this->find_k_close(graph_node->get_state()->get_point(), &second_nodes[0], &second_distances[0], k );

    graph_node->set_index(nodes.size());
    nodes.push_back(graph_node);

    for( unsigned int i=0; i<new_k; i++ )
    {
	    graph_node->add_neighbor( second_nodes[i]->get_index() );
	    second_nodes[i]->add_neighbor( graph_node->get_index() );
    }

}
 
void graph_nearest_neighbors_t::remove_node( state_point_t* state )
{
    const proximity_node_t* graph_node = state->get_proximity_node();
    state->set_proximity_node(nullptr);

    auto neighbors = graph_node->get_neighbors();
    for( unsigned int i=0; i<neighbors.size(); i++ ) {
        nodes[ neighbors[i] ]->delete_neighbor( graph_node->get_index() );
    }

    unsigned int index = graph_node->get_index();
    if( index < nodes.size()-1 )
    {
        nodes[index] = nodes[nodes.size()-1];
        nodes[index]->set_index( index );

        neighbors = nodes[index]->get_neighbors();
        for( unsigned int i=0; i<neighbors.size(); i++ )
            nodes[ neighbors[i] ]->replace_neighbor( nodes.size()-1, index );
    }
    nodes.pop_back();

    delete graph_node;
}

proximity_node_t* graph_nearest_neighbors_t::find_closest( const double* state, double* the_distance ) const
{
    if( nodes.size() == 0 )
        return NULL;
    
    unsigned int nr_samples = sampling_function();
    double min_distance = std::numeric_limits<double>::max();
    int min_index = -1;
    for( unsigned int i=0; i<nr_samples; i++ )
    {
        int index = random_generator.uniform_int_random(0, nodes.size()-1);
        double distance = this->compute_distance(nodes[index], state);
        if( distance < min_distance )
        {
            min_distance = distance;
            min_index = index;
        }
    }

    int old_min_index = min_index;
    do {
        old_min_index = min_index;
        auto neighbors = nodes[min_index]->get_neighbors();
        for( unsigned int j=0; j<neighbors.size(); j++ )
        {
            double distance = this->compute_distance(nodes[neighbors[j]], state);
            if( distance < min_distance )
            {
            min_distance = distance;
            min_index = neighbors[j];
            }
        }
    }
    while( old_min_index != min_index );

    *the_distance = min_distance;
    return nodes[min_index];
}


unsigned int graph_nearest_neighbors_t::find_k_close( const double* state, proximity_node_t** close_nodes, double* distances, unsigned int k )
{
    if( nodes.size() == 0 )
        return 0;

    std::unordered_map<proximity_node_t*,bool> added_nodes;
    
	if(k > MAX_KK)
	{
		// PRX_WARN_S("Trying to return "<<k<<" points when the max is "<<MAX_KK);
		k = MAX_KK;
	}

    if( k < nodes.size() )
    {
        for( unsigned int i=0; i<k; i++ )
        {
            bool exists = true;
            int index;
            while( exists == true )
            {
                index = random_generator.uniform_int_random(0, nodes.size()-1);;
                exists = does_node_exist(added_nodes, nodes[index]);
            }
            close_nodes[i] = nodes[index];
            added_nodes[nodes[index]] = true;
            distances[i] = this->compute_distance(nodes[index], state);
        }
        sort( close_nodes, distances, 0, k-1 );

        unsigned int nr_samples = sampling_function();

        double min_distance = distances[0];
        unsigned int min_index = close_nodes[0]->get_index();
        for( unsigned int i=0; i<nr_samples; i++ )
        {
            int index = random_generator.uniform_int_random(0, nodes.size()-1);;
            if( does_node_exist(added_nodes, nodes[index] ) == false )
            {
                double distance = this->compute_distance(nodes[index], state);
                if( distance < min_distance )
                {
                    min_distance = distance;
                    min_index = index;
                }
            }
        }
        if(min_distance!=distances[0])
        {
            added_nodes.erase(close_nodes[k-1]);
            close_nodes[ k - 1 ] = nodes[min_index];
            added_nodes[nodes[min_index]];
            distances[ k - 1 ] = min_distance;
            resort( close_nodes, distances, k-1 );

        }

        min_index = 0;
        do
        {
            auto neighbors = nodes[ close_nodes[min_index]->get_index() ]->get_neighbors();
            unsigned int lowest_replacement = k;
            for( unsigned int j=0; j<neighbors.size(); j++ )
            {
                if( does_node_exist(added_nodes, nodes[ neighbors[j] ] ) == false )
                {
                    double distance = this->compute_distance(nodes[ neighbors[j] ], state );
                    if( distance < distances[k-1] )
                    {
                        added_nodes.erase(close_nodes[k-1]);
                        close_nodes[k-1] = nodes[ neighbors[j] ];
                        added_nodes[nodes[neighbors[j]]];
                        distances[k-1] = distance;
                        unsigned int test = resort( close_nodes, distances, k-1 );
                        lowest_replacement = (test<lowest_replacement?test:lowest_replacement);
                    }
                }
            }
            if(min_index < lowest_replacement)
                min_index++;
            else
                min_index = lowest_replacement;
        }
        while( min_index < k );

        return k;
    }
    else
    {
        for( unsigned int i=0; i<nodes.size(); i++ )
        {
            close_nodes[i] = nodes[i];
            distances[i] = this->compute_distance(nodes[i], state );
        }

        sort( close_nodes, distances, 0, nodes.size()-1 );
        return nodes.size();
    }
}

std::vector<proximity_node_t*> graph_nearest_neighbors_t::find_delta_close_and_closest( const double* state, double delta )
{
    std::vector<proximity_node_t*> close_nodes;
    if( nodes.size() == 0 )
        return close_nodes;

    std::unordered_map<proximity_node_t*, bool> added_nodes;

    unsigned int nr_samples = sampling_function();
    double min_distance = std::numeric_limits<double>::max();;
    int min_index = -1;
    for( unsigned int i=0; i<nr_samples; i++ )
    {
		int index = random_generator.uniform_int_random(0, nodes.size()-1);;
		double distance = this->compute_distance(nodes[index], state );
		if( distance < min_distance )
		{
		    min_distance = distance;
		    min_index = index;
		}
    }
   
    int old_min_index = min_index;
    do
    {
		old_min_index = min_index;
		auto neighbors = nodes[min_index]->get_neighbors();
		for( unsigned int j=0; j<neighbors.size(); j++ )
		{
		    double distance = this->compute_distance(nodes[ neighbors[j] ], state );
		    if( distance < min_distance )
		    {
				min_distance = distance;
				min_index = neighbors[j];			
		    }
		}
    }
    while( old_min_index != min_index );

    unsigned int nr_points = 0;
    added_nodes[nodes[min_index]];
    close_nodes.push_back(nodes[min_index]);
    nr_points++;
    if( min_distance < delta )
    {
		for( unsigned int counter = 0; counter<nr_points; counter++ )
		{
		    auto neighbors = close_nodes[counter]->get_neighbors();
		    for( unsigned int j=0; j<neighbors.size(); j++ )
		    {
				if( does_node_exist(added_nodes, nodes[ neighbors[j] ]) == false )
				{
				    double distance = this->compute_distance(nodes[ neighbors[j] ], state );
				    if( distance < delta && nr_points < MAX_KK)
				    {
                        close_nodes.push_back(nodes[ neighbors[j] ]);
						added_nodes[close_nodes.back()];
						nr_points++;
				    }
				}
		    }
		}
    }
    return close_nodes;
}

unsigned int graph_nearest_neighbors_t::find_delta_close( const double* state, proximity_node_t** close_nodes, double* distances, double delta )
{
    if( nodes.size() == 0 )
        return 0;

    std::unordered_map<proximity_node_t*,bool> added_nodes;
    unsigned int nr_samples = sampling_function();
    double min_distance = std::numeric_limits<double>::max();;
    int min_index = -1;
    for( unsigned int i=0; i<nr_samples; i++ )
    {
		int index = random_generator.uniform_int_random(0, nodes.size()-1);;
		double distance = this->compute_distance(nodes[index], state );
		if( distance < min_distance )
		{
		    min_distance = distance;
		    min_index = index;
		}
    }
   
    int old_min_index = min_index;
    do
    {
        old_min_index = min_index;
        auto neighbors = nodes[min_index]->get_neighbors();
        for( unsigned int j=0; j<neighbors.size(); j++ )
        {
            double distance = this->compute_distance(nodes[ neighbors[j] ], state );
            if( distance < min_distance )
            {
                min_distance = distance;
                min_index = neighbors[j];
            }
        }
    }
    while( old_min_index != min_index );

    unsigned int nr_points = 0;
    if( min_distance < delta )
    {
        close_nodes[0] = nodes[min_index];
		added_nodes[close_nodes[0]];
        distances[0]   = min_distance;
        nr_points++;
	
        for( unsigned int counter = 0; counter<nr_points; counter++ )
		{
		    auto neighbors = close_nodes[counter]->get_neighbors();
		    for( unsigned int j=0; j<neighbors.size(); j++ )
		    {
				if( does_node_exist(added_nodes, nodes[ neighbors[j] ]) == false )
				{
				    double distance = this->compute_distance(nodes[ neighbors[j] ], state );
				    if( distance < delta && nr_points < MAX_KK)
				    {
						close_nodes[ nr_points ] = nodes[ neighbors[j] ];
						added_nodes[close_nodes[nr_points]];
						distances[ nr_points ] = distance;
						nr_points++;
				    }
				}
		    }
		}
    }
    return nr_points;
}

bool graph_nearest_neighbors_t::does_node_exist(std::unordered_map<proximity_node_t*,bool> const& added_nodes, proximity_node_t* query_node)
{
    return added_nodes.find(query_node)!=added_nodes.end();
}

unsigned int graph_nearest_neighbors_t::sampling_function() const
{
    if( nodes.size() < 1000 )
	    return nodes.size()/5 + 1;
    else
	    return 200 + nodes.size()/500;
}

int graph_nearest_neighbors_t::percolation_threshold()
{
    int k;

    if( nodes.size() > 14)
	    k = 4.25 * log( nodes.size() );
    else 
	    k = nodes.size();
    return k;
}

double graph_nearest_neighbors_t::compute_distance(const proximity_node_t* node, const double* state) const {
    return this->distance_function(node->get_state()->get_point(), state);
}