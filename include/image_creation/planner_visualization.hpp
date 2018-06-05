/**
 * @file planner_visualization.hpp
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
 * @param root Pointer to the root of the search tree.
 * @param last_solution_path Path with the solution to the planning problem.
 * @param projector Object that projects state space points onto image plane.
 * @param start_state Start state in the state space
 * @param goal_state Goal state in the state space
 * @param image_width Width of the drawing.
 * @param image_height Height of the drawing.
 * @param solution_node_diameter Diameter of a node.
 * @param solution_line_width Width of a planning solution path.
 * @param tree_line_width Width of the edges in the planning tree.
 *
 * @return A string with SVG xml that describes planning tree drawing
 */
std::string visualize_tree(
    tree_node_t* root,
    const std::vector<std::vector<double>>& last_solution_path,
    projection_function projector,
    double* start_state, double* goal_state,
    int image_width, int image_height,
    double solution_node_diameter, double solution_line_width, double tree_line_width);

/**
 * @brief Generate an image visualizing the nodes in the tree.
 * @details Generate an image visualizing the nodes in the tree. The nodes will have a grayscale
 * color corresponding to their relative cost to the maximum in the tree.
 *
 * @param root Pointer to the root of the search tree.
 * @param last_solution_path Path with the solution to the planning problem.
 * @param projector Object that projects state space points onto image plane.
 * @param start_state Start state in the state space
 * @param goal_state Goal state in the state space
 * @param image_width Width of the drawing.
 * @param image_height Height of the drawing.
 * @param node_diameter Diameter of nodes
 * @param solution_node_diameter Diameter of nodes that belong to the planning solution
 *
 * @return A string with SVG xml that describes planning nodes drawing
 */
std::string visualize_nodes(
    tree_node_t* root,
    const std::vector<std::vector<double>>& last_solution_path,
    projection_function projector,
    double* start_state, double* goal_state,
    int image_width, int image_height,
    double node_diameter, double solution_node_diameter);


#endif