/**
 * @file planner_visualization.cpp
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

#include "image_creation/svg_image.hpp"
#include "image_creation/planner_visualization.hpp"




/**
 * @brief A helper for visualizing state space point
 * @details A helper for visualizing state space point
 *
 * @param projector Function to project state space point to 2d coordinates
 * @param state State space point
 * @param dims The size of the image.
 *
 * @return svg::Point object
 */
svg::Point visualize_point(projection_function projector, const double* state, svg::Dimensions dims) {
    double x, y;
    std::tie(x, y) = projector(state);
    return {x*dims.width, y*dims.height};
}

/**
 * @brief Creates a single edge geometry with a node's parent.
 * @details Creates a single edge geometry with a node's parent.
 *
 * @param node The target node of the edge.
 * @param projector Function to project state space point to 2d coordinates
 * @param doc The image storage.
 * @param dim The size of the image.
 * @param tree_line_width The width of the planning graph lines
 */
void visualize_edge(tree_node_t* node, projection_function projector, svg::DocumentBody& doc, svg::Dimensions& dim, double tree_line_width)
{
	for (std::list<tree_node_t*>::const_iterator i = node->get_children().begin(); i != node->get_children().end(); ++i)
	{
		svg::Polyline traj_line(svg::Stroke(tree_line_width, svg::Color::Blue));

		traj_line<<visualize_point(projector, node->get_point(), dim);
		traj_line<<visualize_point(projector, (*i)->get_point(), dim);
		doc<<traj_line;

		visualize_edge(*i, projector, doc, dim, tree_line_width);
	}
}

/**
 * @brief Create geometries for visualizing the solution path.
 * @details Create geometries for visualizing the solution path.
 *
 * @param last_solution_path solution path in the state space
 * @param projector Function to project state space point to 2d coordinates
 * @param doc The image storage.
 * @param dim The size of the image.
 * @param solution_line_width The width of the planning solution
 */
void visualize_solution_path(
    const std::vector<std::vector<double>>& last_solution_path,
    projection_function projector,
    svg::DocumentBody& doc, svg::Dimensions& dim,
    double solution_line_width)
{
	if(last_solution_path.size()!=0)
	{
		svg::Polyline traj_line(svg::Stroke(solution_line_width, svg::Color::Black));
		for(unsigned i=0;i<last_solution_path.size();i++)
		{
			traj_line<<visualize_point(projector, &last_solution_path[i][0],dim);
		}
		doc<<traj_line;
	}
}


/**
 * @brief A recursive function for finding the highest cost in the tree.
 * @details A recursive function for finding the highest cost in the tree.
 *
 * @param node The current node being examined.
 * @param max_cost returned max cost of the node
 * @param nodes returns investigated nodes
 */
void get_max_cost(tree_node_t* node, double& max_cost, std::vector<tree_node_t*>& nodes)
{
	nodes.push_back(node);
	if(node->get_cost() > max_cost) {
	    max_cost = node->get_cost();
	}
	for (std::list<tree_node_t*>::const_iterator i = node->get_children().begin(); i != node->get_children().end(); ++i)
	{
		get_max_cost(*i, max_cost, nodes);
	}
}


/**
 * @brief Sorts nodes in the vector
 * @details Sorts nodes in the vector
 *
 * @param nodes nodes (TODO: use vector sort)
 */
void sort(std::vector<tree_node_t*>& nodes)
{
	for(unsigned i=0;i<nodes.size();i++)
	{
		tree_node_t* x = nodes[i];
		unsigned j = i;
		while(j>0 && nodes[j-1]->get_cost() > x->get_cost())
		{
			nodes[j] = nodes[j-1];
			j--;
		}
		nodes[j] = x;
	}
}


/**
 * @brief Creates a single node geometry.
 * @details Creates a single node geometry.
 *
 * @param node The node to visualize.
 * @param projector Function to project state space point to 2d coordinates
 * @param doc The image storage.
 * @param dim The size of the image.
 * @param node_diameter diameter of the node element
 * @param max_cost max cost of the nodes
 */
void visualize_node(
    const tree_node_t* node,
    projection_function projector,
    svg::DocumentBody& doc, svg::Dimensions& dim, double node_diameter, double max_cost)
{

	svg::Circle circle(
	    visualize_point(projector, node->get_point(), dim),
	    node_diameter,
	    svg::Fill( svg::Color((node->get_cost()/max_cost)*255,(node->get_cost()/max_cost)*255,(node->get_cost()/max_cost)*255) ) );
	doc<<circle;
}

/**
 * @brief Create geometries for visualizing the nodes along the solution path.
 * @details Create geometries for visualizing the nodes along the solution path.
 *
 * @param last_solution_path solution path in the state space
 * @param projector Function to project state space point to 2d coordinates
 * @param doc The image storage.
 * @param dim The size of the image.
 * @param solution_node_diameter diameter of solution nodes
 */

void visualize_solution_nodes(
    const std::vector<std::vector<double>>& last_solution_path,
    projection_function projector,
    svg::DocumentBody& doc, svg::Dimensions& dim, double solution_node_diameter)
{

	if(last_solution_path.size()!=0)
	{
		for(unsigned i=0;i<last_solution_path.size();i++)
		{
			svg::Circle circle(visualize_point(projector, &last_solution_path[i][0],dim), solution_node_diameter, svg::Fill( svg::Color(0,255,0) ));
			doc<<circle;
		}
	}
}


/**
 * @copydoc visualize_tree()
 */
std::string visualize_tree(
    tree_node_t* root,
    const std::vector<std::vector<double>>& last_solution_path,
    projection_function projector,
    double* start_state, double* goal_state,
    int image_width, int image_height,
    double solution_node_diameter, double solution_line_width, double tree_line_width)
{
    svg::Dimensions dimensions(image_width, image_height);
    svg::DocumentBody doc(svg::Layout(dimensions, svg::Layout::BottomLeft));

    visualize_edge(root, projector, doc, dimensions, tree_line_width);

	svg::Circle circle(visualize_point(projector, start_state, dimensions), solution_node_diameter, svg::Fill( svg::Color(255,0,0) ));
	doc<<circle;
	svg::Circle circle2(visualize_point(projector, goal_state, dimensions), solution_node_diameter, svg::Fill( svg::Color(0,255,0) ));
	doc<<circle2;

	visualize_solution_path(last_solution_path, projector, doc, dimensions, solution_line_width);

    return doc.toString();
}

/**
 * @copydoc visualize_nodes()
 */
std::string visualize_nodes(
    tree_node_t* root,
    const std::vector<std::vector<double>>& last_solution_path,
    projection_function projector,
    double* start_state, double* goal_state,
    int image_width, int image_height,
    double node_diameter, double solution_node_diameter)
{
    svg::Dimensions dimensions(image_width, image_height);
    svg::DocumentBody doc(svg::Layout(dimensions, svg::Layout::BottomLeft));

    std::vector<tree_node_t*> sorted_nodes;
    double max_cost = 0;
    get_max_cost(root, max_cost, sorted_nodes);
    sort(sorted_nodes);

    for(unsigned i=sorted_nodes.size()-1;i!=0;i--)
    {
	    visualize_node(sorted_nodes[i], projector, doc, dimensions, node_diameter, max_cost);
	}

	svg::Circle circle(visualize_point(projector, start_state,dimensions), solution_node_diameter, svg::Fill( svg::Color(255,0,0) ));
	doc<<circle;
	svg::Circle circle2(visualize_point(projector, goal_state,dimensions), solution_node_diameter, svg::Fill( svg::Color(0,255,0) ));
	doc<<circle2;

	visualize_solution_nodes(last_solution_path, projector, doc, dimensions, solution_node_diameter);

    return doc.toString();
}

