/**
 * @file graph_nearest_neighbors.hpp
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

#ifndef SPARSE_GRAPH_NEIGHBORS_HPP
#define SPARSE_GRAPH_NEIGHBORS_HPP

#include <vector>
#include <unordered_map>
#include <functional>

#include "utilities/random.hpp"

class state_point_t;

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
        proximity_node_t( const state_point_t* st );

        /**
         * Gets the internal node that is represented.
         * @brief Gets the internal node that is represented.
         * @return The internal node.
         */
        const state_point_t* get_state( ) const;

        /**
         * Gets the position of the node in the data structure. Used for fast deletion.
         * @brief Gets the position of the node in the data structure.
         * @return The index value.
         */
        int get_index() const;
        
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
        const std::vector<unsigned int>& get_neighbors() const;

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

    protected:
        /**
         * @brief The node represented.
         */
        const state_point_t* state;

        /**
         * @brief Index in the data structure. Serves as an identifier to other nodes.
         */
        int index;

        /**
         * @brief The neighbor container for this node.
         */
        std::vector<unsigned int> neighbors;
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
unsigned int resort( proximity_node_t** close_nodes, double* distances, unsigned int index );

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
        void add_node( state_point_t* node );
        
        /**
         * @brief Removes a node from the structure.
         * @param node
         */
        void remove_node( state_point_t* node );

        /**
         * Prints the average degree of all vertices in the data structure.
         * @brief Prints the average degree of all vertices in the data structure.
         */
        double average_valence() const;

        /**
         * Returns the closest node in the data structure.
         * @brief Returns the closest node in the data structure.
         * @param state The query point.
         * @param distance The resulting distance between the closest point and the query point.
         * @return The closest point.
         */
        proximity_node_t* find_closest( const double* state, double* distance ) const;
        
        /**
         * Find the k closest nodes to the query point. This is performed using a graph search starting from sqrt(nr_points) random points.
         * @brief Find the k closest nodes to the query point.
         * @param state The query state.
         * @param close_nodes The returned close nodes.
         * @param distances The corresponding distances to the query point.
         * @param k The number to return.
         * @return The number of nodes actually returned.
         */
        unsigned int find_k_close( const double* state, proximity_node_t** close_nodes, double* distances, unsigned int k );
        
        /**
         * Find all nodes within a radius and the closest node. 
         * @brief Find all nodes within a radius and the closest node.
         * @param state The query state.
         * @param close_nodes The returned close nodes.
         * @param distances The corresponding distances to the query point.
         * @param delta The radius to search within.
         * @return The number of nodes returned.
         */
        std::vector<proximity_node_t*> find_delta_close_and_closest( const double* state, double delta );
        
        /**
         * Find all nodes within a radius. 
         * @brief Find all nodes within a radius.
         * @param state The query state.
         * @param close_nodes The returned close nodes.
         * @param distances The corresponding distances to the query point.
         * @param delta The radius to search within.
         * @return The number of nodes returned.
         */
        unsigned int find_delta_close( const double* state, proximity_node_t** close_nodes, double* distances, double delta );

        /**
         * Set distance function for NN structure
         * @brief Set distance function for NN structure
         * @param new_distance distance function that returns distance between state point
         */
        void set_distance(std::function<double(const double*, const double*)> new_distance)
        {
            this->distance_function = new_distance;
        }

    protected:

        /**
         * @brief Distance function between state space points
         */
        std::function<double(const double*, const double*)> distance_function;

        /**
         * Helper function for determining existance in a list.
         * @brief Helper function for determining existance in a list.
         * @param query_node The node to search for.
         * @return If query_node exists in node_list.
         */
        bool does_node_exist(std::unordered_map<proximity_node_t*,bool> const& added_nodes, proximity_node_t* query_node);
        
        /**
         * Determine the number of nodes to sample for initial populations in queries.
         * @brief Determine the number of nodes to sample for initial populations in queries.
         * @return The number of random nodes to initially select.
         */
        unsigned int sampling_function() const;
        
        /**
         * @brief Given the number of nodes, get the number of neighbors required for connectivity (in the limit).
         */
        int percolation_threshold();
        
        /**
         * @brief The nodes being stored.
         */
        std::vector<proximity_node_t*> nodes;

        /**
         * @brief Temporary storage for query functions.
         */
        std::vector<proximity_node_t*> second_nodes;
        
        /**
         * @brief Temporary storage for query functions.
         */
        std::vector<double> second_distances;
    private:
        /**
         * Helper function to compute distance between NN node and state space point
         * @brief Helper function to compute distance between NN node and state space point
         * @node node NN node
         * @state state State space point
         * @return distance between state space point, represented by node and state
         */
        double compute_distance(const proximity_node_t* node, const double* state) const;

        /**
         * @brief Random number generator
         */
        mutable RandomGenerator random_generator;
};

#endif 
