/**
 * @file graph_nearest_neighbors.hpp
 * 
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2014, Rutgers the State University of New Jersey, New Brunswick  
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 * 
 * Authors: Zakary Littlefield, Kostas Bekris 
 * 
 */

#ifndef SPARSE_GRAPH_NEIGHBORS_HPP
#define SPARSE_GRAPH_NEIGHBORS_HPP

#include <boost/unordered_map.hpp>

class tree_node_t;
class system_t;

#define INIT_NODE_SIZE    1000
#define INIT_CAP_NEIGHBORS 200
#define MAX_KK          2000

/**
 * @brief A node for the nearest neighbor structure.
 * @details A node for the nearest neighbor structure.
 * 
 */
class proximity_node_t
{
    public:
        /**
         * @brief Constructor
         * @param st The node to store.
         */
        proximity_node_t( const tree_node_t* st );
        virtual ~proximity_node_t();

        /**
         * Determines distance with another node.
         * @brief Determines distance with another node.
         * @param st The node to determine distance with.
         * @return The distance value.
         */
        double distance ( const tree_node_t* st ); 
        
        /**
         * Determines distance with another node.
         * @brief Determines distance with another node.
         * @param other The node to determine distance with.
         * @return The distance value.
         */
        double distance ( const proximity_node_t* other );

        /**
         * Gets the internal node that is represented.
         * @brief Gets the internal node that is represented.
         * @return The internal node.
         */
        const tree_node_t* get_state( );

        /**
         * Gets the position of the node in the data structure. Used for fast deletion.
         * @brief Gets the position of the node in the data structure.
         * @return The index value.
         */
        int get_index();
        
        /**
         * Sets the position of the node in the data structure. Used for fast deletion.
         * @brief Sets the position of the node in the data structure.
         * @param indx The index value.
         */
        void set_index( int indx );

        /**
         * Returns the stored neighbors.
         * @brief Returns the stored neighbors.
         * @param nr_neigh Storage for the number of neighbors returned.
         * @return The neighbor indices.
         */
        unsigned int* get_neighbors( int* nr_neigh );

        /**
         * Adds a node index into this node's neighbor list.
         * @brief Adds a node index into this node's neighbor list.
         * @param node The index to add.
         */
        void add_neighbor( unsigned int node );

        /**
         * Deletes a node index from this node's neighbor list.
         * @brief Deletes a node index from this node's neighbor list.
         * @param node The index to delete.
         */
        void delete_neighbor( unsigned int node );

        /**
         * Replaces a node index from this node's neighbor list.
         * @brief Replaces a node index from this node's neighbor list.
         * @param prev The index to look for.
         * @param new_index The index to replace with.
         */
        void replace_neighbor( unsigned prev, int new_index );

        system_t* system;
    protected:
        /**
         * @brief The node represented.
         */
        const tree_node_t* state; 

        /**
         * @brief Index in the data structure. Serves as an identifier to other nodes.
         */
        int index;

        /**
         * @brief The max number of neighbors.
         */
        int cap_neighbors;
        
        /**
         * @brief The current number of neighbors.
         */
        int nr_neighbors;
        
        /**
         * @brief The neighbor list for this node.
         */
        unsigned int* neighbors;
};



/**
 * Sorts a list of proximity_node_t's. Performed using a quick sort operation. 
 * @param close_nodes The list to sort.
 * @param distances The distances that determine the ordering.
 * @param low The lower index.
 * @param high The upper index.
 */
void sort( proximity_node_t** close_nodes, double* distances, int low, int high );

/**
 * Performs sorting over a list of nodes. Assumes all nodes before index are sorted.
 * @param close_nodes The list to sort.
 * @param distances The distances that determine the ordering.
 * @param index The index to start from.
 */
int resort( proximity_node_t** close_nodes, double* distances, int index );

/**
 * A proximity structure based on graph literature. Each node maintains a list of neighbors.
 * When performing queries, the graph is traversed to determine other locally close nodes.
 * @brief A proximity structure based on graph literature.
 * @author Kostas Bekris
 */
class graph_nearest_neighbors_t
{
    public:
        /**
         * @brief Constructor
         * @param state The first node to add to the structure.
         */
        graph_nearest_neighbors_t();
        ~graph_nearest_neighbors_t();

        /**
         * Adds a node to the proximity structure
         * @brief Adds a node to the proximity structure
         * @param node The node to insert.
         */
        void add_node( proximity_node_t* node );
        
        /**
         * @brief Removes a node from the structure.
         * @param node
         */
        void remove_node( proximity_node_t* node );

        /**
         * Prints the average degree of all vertices in the data structure.
         * @brief Prints the average degree of all vertices in the data structure.
         */
        void average_valence();

        /**
         * Returns the closest node in the data structure.
         * @brief Returns the closest node in the data structure.
         * @param state The query point.
         * @param distance The resulting distance between the closest point and the query point.
         * @return The closest point.
         */
        proximity_node_t* find_closest( tree_node_t* state, double* distance );          
        
        /**
         * Find the k closest nodes to the query point. This is performed using a graph search starting from sqrt(nr_points) random points.
         * @brief Find the k closest nodes to the query point.
         * @param state The query state.
         * @param close_nodes The returned close nodes.
         * @param distances The corresponding distances to the query point.
         * @param k The number to return.
         * @return The number of nodes actually returned.
         */
        int find_k_close( tree_node_t* state, proximity_node_t** close_nodes, double* distances, int k );
        
        /**
         * Find all nodes within a radius and the closest node. 
         * @brief Find all nodes within a radius and the closest node.
         * @param state The query state.
         * @param close_nodes The returned close nodes.
         * @param distances The corresponding distances to the query point.
         * @param delta The radius to search within.
         * @return The number of nodes returned.
         */
        int find_delta_close_and_closest( tree_node_t* state, proximity_node_t** close_nodes, double* distances, double delta );
        
        /**
         * Find all nodes within a radius. 
         * @brief Find all nodes within a radius.
         * @param state The query state.
         * @param close_nodes The returned close nodes.
         * @param distances The corresponding distances to the query point.
         * @param delta The radius to search within.
         * @return The number of nodes returned.
         */
        int find_delta_close( tree_node_t* state, proximity_node_t** close_nodes, double* distances, double delta );


        void set_system(system_t* new_system)
        {
            system = new_system;
        }

    protected:

        system_t* system;

        /**
         * Helper function for determining existance in a list.
         * @brief Helper function for determining existance in a list.
         * @param query_node The node to search for.
         * @param node_list The list to search.
         * @param list_size The size of the list.
         * @return If query_node exists in node_list.
         */
        bool does_node_exist( proximity_node_t* query_node, proximity_node_t** node_list, int list_size );
        
        /**
         * Determine the number of nodes to sample for initial populations in queries.
         * @brief Determine the number of nodes to sample for initial populations in queries.
         * @return The number of random nodes to initially select.
         */
        int sampling_function();
        
        /**
         * Given the number of nodes, get the number of neighbors required for connectivity (in the limit).
         * @brief Given the number of nodes, get the number of neighbors required for connectivity (in the limit).
         * @return 
         */
        int percolation_threshold();
        
        /**
         * @brief The nodes being stored.
         */
        proximity_node_t** nodes;
        
        /**
         * @brief The current number of nodes being stored.
         */
        int nr_nodes;
        
        /**
         * @brief The maximum number of nodes that can be stored. 
         */
        int cap_nodes;

        /**
         * @brief Temporary storage for query functions.
         */
        proximity_node_t** second_nodes;
        
        /**
         * @brief Temporary storage for query functions.
         */
        double* second_distances;

        boost::unordered_map<proximity_node_t*,bool> added_nodes;

};

#endif 
