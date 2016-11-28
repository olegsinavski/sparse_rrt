
#ifndef PLANNER_VISUALIZATION_HPP
#define PLANNER_VISUALIZATION_HPP

#include "motion_planners/tree_node.hpp"
#include "systems/system.hpp"


/**
 * @brief Generate an image visualizing the tree.
 * @details Generate an image visualizing the tree.
 *
 * @param image_counter A subscript for the image file name. Allows for multiple image output.
 */
void visualize_tree(
    tree_node_t* root,
    const std::vector<tree_node_t*>& last_solution_path,
    system_t* system,
    double* start_state, double* goal_state,
    int image_counter,
    int image_width, int image_height,
    double solution_node_diameter, double solution_line_width, double tree_line_width);

/**
 * @brief Generate an image visualizing the nodes in the tree.
 * @details Generate an image visualizing the nodes in the tree. The nodes will have a grayscale
 * color corresponding to their relative cost to the maximum in the tree.
 *
 * @param image_counter A subscript for the image file name. Allows for multiple image output.
 */
void visualize_nodes(
    tree_node_t* root,
    const std::vector<tree_node_t*>& last_solution_path,
    system_t* system,
    double* start_state, double* goal_state,
    int image_counter,
    int image_width, int image_height,
    double node_diameter, double solution_node_diameter);



#endif