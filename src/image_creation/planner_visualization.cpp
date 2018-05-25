
#include "image_creation/svg_image.hpp"
#include "image_creation/planner_visualization.hpp"


/**
 * @brief Creates a single edge geometry with a node's parent.
 * @details Creates a single edge geometry with a node's parent.
 *
 * @param node The target node of the edge.
 * @param doc The image storage.
 * @param dim The size of the image.
 */
void visualize_edge(tree_node_t* node, system_t* system, svg::Document& doc, svg::Dimensions& dim, double tree_line_width)
{
	for (std::list<tree_node_t*>::const_iterator i = node->children.begin(); i != node->children.end(); ++i)
	{
		svg::Polyline traj_line(svg::Stroke(tree_line_width, svg::Color::Blue));

		traj_line<<system->visualize_point(node->get_point(),dim);
		traj_line<<system->visualize_point((*i)->get_point(),dim);
		doc<<traj_line;

		visualize_edge(*i, system, doc, dim, tree_line_width);
	}
}

/**
 * @brief Create geometries for visualizing the solution path.
 * @details Create geometries for visualizing the solution path.
 *
 * @param doc The image storage.
 * @param dim The size of the image.
 */
void visualize_solution_path(const std::vector<std::vector<double>>& last_solution_path, system_t* system, svg::Document& doc, svg::Dimensions& dim, double solution_line_width)
{
	if(last_solution_path.size()!=0)
	{
		svg::Polyline traj_line(svg::Stroke(solution_line_width, svg::Color::Black));
		for(unsigned i=0;i<last_solution_path.size();i++)
		{
			traj_line<<system->visualize_point(&last_solution_path[i][0],dim);
		}
		doc<<traj_line;
	}
}



/**
 * @brief A recursive function for finding the highest cost in the tree.
 * @details A recursive function for finding the highest cost in the tree.
 *
 * @param node The current node being examined.
 */
void get_max_cost(tree_node_t* node, double& max_cost, std::vector<tree_node_t*>& nodes)
{
	nodes.push_back(node);
	if(node->cost > max_cost)
		max_cost = node->cost;
	for (std::list<tree_node_t*>::const_iterator i = node->children.begin(); i != node->children.end(); ++i)
	{
		get_max_cost(*i, max_cost, nodes);
	}
}


void sort(std::vector<tree_node_t*>& nodes)
{
	for(unsigned i=0;i<nodes.size();i++)
	{
		tree_node_t* x = nodes[i];
		unsigned j = i;
		while(j>0 && nodes[j-1]->cost > x->cost)
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
 * @param doc The image storage.
 * @param dim The size of the image.
 */
void visualize_node(
    tree_node_t* node,
    system_t* system,
    svg::Document& doc, svg::Dimensions& dim, double node_diameter, double max_cost)
{

	svg::Circle circle(system->visualize_point(node->get_point(), dim), node_diameter, svg::Fill( svg::Color((node->cost/max_cost)*255,(node->cost/max_cost)*255,(node->cost/max_cost)*255) ) );
	doc<<circle;
	// for (std::list<tree_node_t*>::iterator i = node->children.begin(); i != node->children.end(); ++i)
	// {
	// 	visualize_node(*i,doc,dim);
	// }

}

/**
 * @brief Create geometries for visualizing the nodes along the solution path.
 * @details Create geometries for visualizing the nodes along the solution path.
 *
 * @param doc The image storage.
 * @param dim The size of the image.
 */

void visualize_solution_nodes(
    const std::vector<std::vector<double>>& last_solution_path,
    system_t* system,
    svg::Document& doc, svg::Dimensions& dim, double solution_node_diameter)
{

	if(last_solution_path.size()!=0)
	{
		for(unsigned i=0;i<last_solution_path.size();i++)
		{
			svg::Circle circle(system->visualize_point(&last_solution_path[i][0],dim), solution_node_diameter, svg::Fill( svg::Color(0,255,0) ));
			doc<<circle;
		}
	}
}



void visualize_tree(
    std::string const & file_name,
    tree_node_t* root,
    const std::vector<std::vector<double>>& last_solution_path,
    system_t* system,
    double* start_state, double* goal_state,
    int image_width, int image_height,
    double solution_node_diameter, double solution_line_width, double tree_line_width)
{
    svg::Dimensions dimensions(image_width, image_height);
    svg::Document doc(file_name, svg::Layout(dimensions, svg::Layout::BottomLeft));

    visualize_edge(root, system, doc, dimensions, tree_line_width);

	svg::Circle circle(system->visualize_point(start_state, dimensions), solution_node_diameter, svg::Fill( svg::Color(255,0,0) ));
	doc<<circle;
	svg::Circle circle2(system->visualize_point(goal_state, dimensions), solution_node_diameter, svg::Fill( svg::Color(0,255,0) ));
	doc<<circle2;

	visualize_solution_path(last_solution_path, system, doc, dimensions, solution_line_width);
    system->visualize_obstacles(doc,dimensions);

    doc.save();
}

void visualize_nodes(
    std::string const & file_name,
    tree_node_t* root,
    const std::vector<std::vector<double>>& last_solution_path,
    system_t* system,
    double* start_state, double* goal_state,
    int image_width, int image_height, double node_diameter, double solution_node_diameter)
{
    svg::Dimensions dimensions(image_width, image_height);
    svg::Document doc(file_name, svg::Layout(dimensions, svg::Layout::BottomLeft));

    std::vector<tree_node_t*> sorted_nodes;
    double max_cost = 0;
    get_max_cost(root, max_cost, sorted_nodes);
    sort(sorted_nodes);

    for(unsigned i=sorted_nodes.size()-1;i!=0;i--)
    {
	    visualize_node(sorted_nodes[i], system, doc, dimensions, node_diameter, max_cost);
	}

	svg::Circle circle(system->visualize_point(start_state,dimensions), solution_node_diameter, svg::Fill( svg::Color(255,0,0) ));
	doc<<circle;
	svg::Circle circle2(system->visualize_point(goal_state,dimensions), solution_node_diameter, svg::Fill( svg::Color(0,255,0) ));
	doc<<circle2;

	visualize_solution_nodes(last_solution_path, system, doc, dimensions, solution_node_diameter);
    system->visualize_obstacles(doc,dimensions);

    doc.save();
}

