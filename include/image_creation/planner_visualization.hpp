
#ifndef PLANNER_VISUALIZATION_HPP
#define PLANNER_VISUALIZATION_HPP

#include "motion_planners/tree_node.hpp"
#include "systems/system.hpp"
#include <functional>

typedef typename std::function<std::tuple<double, double>(const double*)> projection_function;

/**
 * @brief Generate an image visualizing the tree.
 * @details Generate an image visualizing the tree.
 *
 * @param image_counter A subscript for the image file name. Allows for multiple image output.
 */
std::tuple<std::string, std::string, std::string> visualize_tree(
    tree_node_t* root,
    const std::vector<std::vector<double>>& last_solution_path,
    projection_function projector,
    system_t* system,
    double* start_state, double* goal_state,
    int image_width, int image_height,
    double solution_node_diameter, double solution_line_width, double tree_line_width);

/**
 * @brief Generate an image visualizing the nodes in the tree.
 * @details Generate an image visualizing the nodes in the tree. The nodes will have a grayscale
 * color corresponding to their relative cost to the maximum in the tree.
 *
 * @param image_counter A subscript for the image file name. Allows for multiple image output.
 */
std::tuple<std::string, std::string, std::string> visualize_nodes(
    tree_node_t* root,
    const std::vector<std::vector<double>>& last_solution_path,
    projection_function projector,
    system_t* system,
    double* start_state, double* goal_state,
    int image_width, int image_height,
    double node_diameter, double solution_node_diameter);


#endif