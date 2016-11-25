/**
 * @file grpah_nearest_neighbor.cpp
 * 
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2014, Rutgers the State University of New Jersey, New Brunswick  
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 * 
 * Authors: Zakary Littlefield, Kostas Bekris 
 * 
 */

#include "nearest_neighbors/graph_nearest_neighbors.hpp"
#include "motion_planners/tree_node.hpp"
#include "systems/system.hpp"

proximity_node_t::proximity_node_t( const tree_node_t* st )
{
    system = NULL;
    state = st;

    neighbors = (unsigned int*)malloc(INIT_CAP_NEIGHBORS*sizeof(unsigned int));
    nr_neighbors = 0;
    cap_neighbors = INIT_CAP_NEIGHBORS;
}

proximity_node_t::~proximity_node_t()
{
    free( neighbors );
}

double proximity_node_t::distance ( const tree_node_t* st )
{
    return system->distance(state->point,st->point);
}

double proximity_node_t::distance ( const proximity_node_t* other )
{
    return system->distance(state->point,other->state->point);
}

const tree_node_t* proximity_node_t::get_state( )
{
    return state;
}

int proximity_node_t::get_index()
{
    return index;
}

void proximity_node_t::set_index( int indx )
{
    index = indx;
}

unsigned int* proximity_node_t::get_neighbors( int* nr_neigh )
{
    *nr_neigh = nr_neighbors;
    return neighbors;
}

void proximity_node_t::add_neighbor( unsigned int nd )
{
    if( nr_neighbors >= cap_neighbors-1 )
    {
	cap_neighbors = 2*cap_neighbors;
	neighbors = (unsigned int*)realloc( neighbors, cap_neighbors*sizeof(unsigned int));
    }
    neighbors[nr_neighbors] = nd;
    nr_neighbors++;
}

void proximity_node_t::delete_neighbor( unsigned int nd )
{
    int index;
    for( index=0; index<nr_neighbors; index++ )
    {
	if( neighbors[index] == nd )
	    break;
    }
    assert( index < nr_neighbors );

    for( int i=index; i<nr_neighbors-1; i++ )
	neighbors[i] = neighbors[i+1];
    nr_neighbors--;
}

void proximity_node_t::replace_neighbor( unsigned prev, int new_index )
{
    int index;
    for( index=0; index<nr_neighbors; index++ )
    {
	if( neighbors[index] == prev )
	    break;
    }
    assert( index < nr_neighbors );

    neighbors[index] = new_index;
}


////////////////////////////////////////
////// SORTING
///////////////////////////////////////

void sort( proximity_node_t** close_nodes, double* distances, int low, int high )
{ 
    if( low < high )
    {
	int left, right, pivot;
	double pivot_distance = distances[low];
	proximity_node_t* pivot_node = close_nodes[low];
	pivot = left = low;
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

int resort( proximity_node_t** close_nodes, double* distances, int index )
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
{
	added_nodes.clear();

    nodes = (proximity_node_t**)malloc(INIT_NODE_SIZE*sizeof(proximity_node_t*));
    nr_nodes = 0;
    cap_nodes = INIT_NODE_SIZE;

    second_nodes = (proximity_node_t**)malloc( MAX_KK *sizeof(proximity_node_t*));
    second_distances = (double*)malloc( MAX_KK *sizeof(double));
}

graph_nearest_neighbors_t::~graph_nearest_neighbors_t()
{
    free( nodes );

    free( second_nodes );
    free( second_distances);
}

void graph_nearest_neighbors_t::average_valence()
{
    int nr_neigh;
    unsigned int* neighs;
    double all_neighs = 0;
    for( int i=0; i<nr_nodes; i++ )
    {
	neighs = nodes[i]->get_neighbors( &nr_neigh );
	all_neighs += nr_neigh;
    }
    all_neighs /= (double)nr_nodes;
}

void graph_nearest_neighbors_t::add_node( proximity_node_t* graph_node )
{    
	graph_node->system = system;
    int k = percolation_threshold();

    int new_k = find_k_close( (tree_node_t*)(graph_node->get_state()), second_nodes, second_distances, k );

    if( nr_nodes >= cap_nodes-1 )
    {
	cap_nodes = 2 * cap_nodes;
	nodes = (proximity_node_t**)realloc(nodes, cap_nodes*sizeof(proximity_node_t*));
    }
    nodes[nr_nodes] = graph_node;
    
    
    graph_node->set_index(nr_nodes);
    nr_nodes++;

    for( int i=0; i<new_k; i++ )
    {
	graph_node->add_neighbor( second_nodes[i]->get_index() );
	second_nodes[i]->add_neighbor( graph_node->get_index() );
    }

}
 
void graph_nearest_neighbors_t::remove_node( proximity_node_t* graph_node )
{
    int nr_neighbors;
    unsigned int* neighbors = graph_node->get_neighbors( &nr_neighbors );
    for( int i=0; i<nr_neighbors; i++ )
	nodes[ neighbors[i] ]->delete_neighbor( graph_node->get_index() );

    int index = graph_node->get_index();
    if( index < nr_nodes-1 )
    {
	nodes[index] = nodes[nr_nodes-1];
	nodes[index]->set_index( index );
	
	neighbors = nodes[index]->get_neighbors( &nr_neighbors );
	for( int i=0; i<nr_neighbors; i++ )
	    nodes[ neighbors[i] ]->replace_neighbor( nr_nodes-1, index ); 
    }
    nr_nodes--;

    if( nr_nodes < (cap_nodes-1)/2 )
    {
	cap_nodes *= 0.5;
	nodes = (proximity_node_t**)realloc(nodes,cap_nodes*sizeof(proximity_node_t*));
    }
}

proximity_node_t* graph_nearest_neighbors_t::find_closest( tree_node_t* state, double* the_distance )
{
    if( nr_nodes == 0 )
        return NULL;
    
    int nr_samples = sampling_function();
    double min_distance = std::numeric_limits<double>::max();
    int min_index = -1;
    for( int i=0; i<nr_samples; i++ )
    {
	int index = rand() % nr_nodes;
	double distance = nodes[index]->distance( state );
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
	int nr_neighbors;
	unsigned int* neighbors = nodes[min_index]->get_neighbors( &nr_neighbors );
	for( int j=0; j<nr_neighbors; j++ )
	{
	    double distance = nodes[ neighbors[j] ]->distance( state );
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


int graph_nearest_neighbors_t::find_k_close( tree_node_t* state, proximity_node_t** close_nodes, double* distances, int k )
{
    if( nr_nodes == 0 )
        return 0;
    
	added_nodes.clear();
    
	if(k > MAX_KK)
	{
		// PRX_WARN_S("Trying to return "<<k<<" points when the max is "<<MAX_KK);
		k = MAX_KK;
	}

    if( k < nr_nodes )
    {
	for( int i=0; i<k; i++ )
	{
	    bool exists = true;
	    int index;
	    while( exists == true )
	    {
		index = rand() % nr_nodes;
		exists = does_node_exist( nodes[index], close_nodes, i );
	    }
	    close_nodes[i] = nodes[index];
	    added_nodes[nodes[index]] = true;
	    distances[i] = nodes[index]->distance( state );
	}
	sort( close_nodes, distances, 0, k-1 );
        
	int nr_samples = sampling_function();

	double min_distance = distances[0];
	int min_index = close_nodes[0]->get_index();
	for( int i=0; i<nr_samples; i++ )
	{
	    int index = rand() % nr_nodes;
	    if( does_node_exist( nodes[index], close_nodes, k ) == false )
	    {
		double distance = nodes[index]->distance( state );
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
	int last_index = close_nodes[k-1]->get_index();
        
	int old_last_index;
        
        
	do
	{
	    old_last_index = last_index;
            int nr_neighbors;
            unsigned int* neighbors = nodes[ close_nodes[min_index]->get_index() ]->get_neighbors( &nr_neighbors );
            int lowest_replacement = k;
            for( int j=0; j<nr_neighbors; j++ )
            {
                if( does_node_exist( nodes[ neighbors[j] ], close_nodes, k ) == false )
                {
                    double distance = nodes[ neighbors[j] ]->distance( state );
                    if( distance < distances[k-1] )
                    {
                    	added_nodes.erase(close_nodes[k-1]);
                        close_nodes[k-1] = nodes[ neighbors[j] ];
                        added_nodes[nodes[neighbors[j]]];
                        distances[k-1] = distance;
                        int test = resort( close_nodes, distances, k-1 );
                        lowest_replacement = (test<lowest_replacement?test:lowest_replacement);
                        last_index = close_nodes[k-1]->get_index();
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
	for( int i=0; i<nr_nodes; i++ )
	{
	    close_nodes[i] = nodes[i];
	    distances[i] = nodes[i]->distance( state );
	}
        
	sort( close_nodes, distances, 0, nr_nodes-1 );
	return nr_nodes;
    }
}

int graph_nearest_neighbors_t::find_delta_close_and_closest( tree_node_t* state, proximity_node_t** close_nodes, double* distances, double delta )
{
    if( nr_nodes == 0 )
        return 0;
    
	added_nodes.clear();
    int nr_samples = sampling_function();
    double min_distance = 99999999;
    int min_index = -1;
    for( int i=0; i<nr_samples; i++ )
    {
		int index = rand() % nr_nodes;
		double distance = nodes[index]->distance( state );
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
		int nr_neighbors;
		unsigned int* neighbors = nodes[min_index]->get_neighbors( &nr_neighbors );
		for( int j=0; j<nr_neighbors; j++ )
		{
		    double distance = nodes[ neighbors[j] ]->distance( state );
		    if( distance < min_distance )
		    {
				min_distance = distance;
				min_index = neighbors[j];			
		    }
		}
    }
    while( old_min_index != min_index );

    int nr_points = 0;
    added_nodes[nodes[min_index]];
    close_nodes[0] = nodes[min_index];
    distances[0]   = min_distance;
    nr_points++;
    if( min_distance < delta )
    {
		for( int counter = 0; counter<nr_points; counter++ )
		{
		    int nr_neighbors;
		    unsigned int* neighbors = close_nodes[counter]->get_neighbors( &nr_neighbors );	
		    for( int j=0; j<nr_neighbors; j++ )
		    {
				if( does_node_exist( nodes[ neighbors[j] ], close_nodes, nr_points ) == false )
				{
				    double distance = nodes[ neighbors[j] ]->distance( state );
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

int graph_nearest_neighbors_t::find_delta_close( tree_node_t* state, proximity_node_t** close_nodes, double* distances, double delta )
{
    if( nr_nodes == 0 )
        return 0;
    
	added_nodes.clear();
    int nr_samples = sampling_function();
    double min_distance = 99999999;
    int min_index = -1;
    for( int i=0; i<nr_samples; i++ )
    {
		int index = rand() % nr_nodes;
		double distance = nodes[index]->distance( state );
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
	int nr_neighbors;
	unsigned int* neighbors = nodes[min_index]->get_neighbors( &nr_neighbors );
	for( int j=0; j<nr_neighbors; j++ )
	{
	    double distance = nodes[ neighbors[j] ]->distance( state );
	    if( distance < min_distance )
	    {
			min_distance = distance;
			min_index = neighbors[j];			
	    }
	}
    }
    while( old_min_index != min_index );

    int nr_points = 0;
    if( min_distance < delta )
    {
        close_nodes[0] = nodes[min_index];
		added_nodes[close_nodes[0]];
        distances[0]   = min_distance;
        nr_points++;
	
        for( int counter = 0; counter<nr_points; counter++ )
		{
		    int nr_neighbors;
		    unsigned int* neighbors = close_nodes[counter]->get_neighbors( &nr_neighbors );	
		    for( int j=0; j<nr_neighbors; j++ )
		    {
				if( does_node_exist( nodes[ neighbors[j] ], close_nodes, nr_points ) == false )
				{
				    double distance = nodes[ neighbors[j] ]->distance( state );
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

bool graph_nearest_neighbors_t::does_node_exist( proximity_node_t* query_node, proximity_node_t** node_list, int list_size )
{
    bool exists = false;

    exists = added_nodes.find(query_node)!=added_nodes.end();
 //    for( int q=0; q<list_size && exists == false; q++ )
 //    {
	// if( query_node == node_list[q] )
	//     exists = true;
 //    }
    return exists;
}

int graph_nearest_neighbors_t::sampling_function()
{
    if( nr_nodes < 1000 )
	return nr_nodes/5 + 1;
    else
	return 200 + nr_nodes/500;
}

int graph_nearest_neighbors_t::percolation_threshold()
{
    int k;

    if( nr_nodes > 14)
	k = 4.25 * log( nr_nodes );
    else 
	k = nr_nodes;
    return k;
}