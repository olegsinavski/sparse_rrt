/**
 * @file parameter_reader.cpp
 * 
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2014, Rutgers the State University of New Jersey, New Brunswick  
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 * 
 * Authors: Zakary Littlefield, Kostas Bekris 
 * 
 */

#include "utilities/parameter_reader.hpp"
#include <iostream>
#include <fstream>
#include <sstream>

namespace params
{
	double integration_step;
	std::string stopping_type;
	double stopping_check;
	std::string stats_type;
	double stats_check;
	unsigned min_time_steps;
	unsigned max_time_steps;
	int random_seed;
	double sst_delta_near;
	double sst_delta_drain;
	std::string planner;
	std::string system;
	double* start_state;
	double* goal_state;
	double goal_radius;
	bool intermediate_visualization;


	//Parameters for image output.
	double tree_line_width;
	double solution_line_width;
	int image_width;
	int image_height;
	double node_diameter;
	int bacon;
	double solution_node_diameter;
}

#include <boost/program_options.hpp>

namespace po = boost::program_options;

void read_parameters(int ac, char* av[])
{

	std::string config_file_name; 
	po::options_description opt_desc("Options"); 
	opt_desc.add_options() 
	("help","Print available options.") ("config", po::value< std::string >( &config_file_name )->default_value("../input/default.cfg"), 
	"The name of a file to read for options (default is ../input/default.cfg). Command-line options" 
	" override the ones in the config file. A config file may contain lines with syntax" 
	"\n'long_option_name = value'\nand comment lines that begin with '#'." )
	("integration_step",po::value<double>(&params::integration_step),"Integration step for propagations.") 
	("stopping_type",po::value<std::string>(&params::stopping_type),"Condition for terminating planner (iterations or time).") 
	("stopping_check",po::value<double>(&params::stopping_check),"Amount of time or iterations to execute.") 
	("stats_type",po::value<std::string>(&params::stats_type),"Condition for printing statistics of a planner (iterations or time).") 
	("stats_check",po::value<double>(&params::stats_check),"Frequency of statistics gathering.") 
	("intermediate_visualization",po::value<bool>(&params::intermediate_visualization)->default_value(false),"Flag denoting generating images during statistics gathering.") 
	("min_time_steps",po::value<unsigned>(&params::min_time_steps),"Minimum number of simulation steps per local planner propagation.") 
	("max_time_steps",po::value<unsigned>(&params::max_time_steps),"Maximum number of simulation steps per local planner propagation.") 
	("bacon",po::value<int>(&params::bacon)->default_value(0),"bacon strips and bacon strips and bacon strips") 
	("random_seed",po::value<int>(&params::random_seed),"Random seed for the planner.") 
	("sst_delta_near",po::value<double>(&params::sst_delta_near),"The radius for BestNear in SST.") 
	("sst_delta_drain",po::value<double>(&params::sst_delta_drain),"The radius for witness nodes in SST.") 
	("planner",po::value<std::string>(&params::planner),"A string for the planner to run.") 
	("system",po::value<std::string>(&params::system),"A string for the system to plan for.") 
	("start_state", po::value<std::string >(), "The given start state. Input is in the format of \"0 0\"")
	("goal_state", po::value<std::string >(), "The given goal state. Input is in the format of \"0 0\"")
	("goal_radius",po::value<double>(&params::goal_radius),"The radius for the goal region.") 
	("tree_line_width",po::value<double>(&params::tree_line_width),"Line thickness for tree visualization.") 
	("solution_line_width",po::value<double>(&params::solution_line_width),"Line thickness for solution path.") 
	("image_width",po::value<int>(&params::image_width),"Width of output images.") 
	("image_height",po::value<int>(&params::image_height),"Height of output images.") 
	("node_diameter",po::value<double>(&params::node_diameter),"Diameter of visualized nodes in output images.") 
	("solution_node_diameter",po::value<double>(&params::solution_node_diameter),"Diameter of nodes along solution path in output images.") 
	;

    po::variables_map varmap;
	po::store(po::parse_command_line(ac,av,opt_desc),varmap); 
	po::notify( varmap ); 

	if (varmap.count("help")) 
	{
    	std::cout << opt_desc << "\n";
	    exit(0);
	}

	if( varmap.count("config") ) 
	{ 
	  std::cout << "Loading options from " << config_file_name; 
	  std::cout.flush(); 
	  std::ifstream ifs( config_file_name.c_str() ); 
	  if( !ifs.is_open() ) 
	   std::cout << "no such file." << std::endl; 
	  else 
	  { 
	   po::store( po::parse_config_file( ifs, opt_desc ), varmap ); 
	   po::notify( varmap ); 
	   std::cout << " done." << std::endl; 
	  } 
	} 
	
	if(params::bacon>0)
	{
		std::cout<<"bacon strips";
		for(int i=1;i<params::bacon;i++)
		{
			std::cout<<" and \nbacon strips";
		}
		std::cout<<"\n";
	}

	std::vector<double> state;
	if (varmap.count("start_state")) 
	{
		std::stringstream stream(varmap["start_state"].as<std::string>());
		double n; while(stream >> n) {state.push_back(n);}
		// state = varmap["start_state"].as<std::vector<double> >();
		params::start_state = new double[state.size()];
		for(unsigned i=0;i<state.size();i++)
		{
			params::start_state[i] = state[i];
		}
	}
	state.clear();
	if (varmap.count("goal_state")) 
	{
		std::stringstream stream(varmap["goal_state"].as<std::string>());
		double n; while(stream >> n) {state.push_back(n);}
		params::goal_state = new double[state.size()];
		for(unsigned i=0;i<state.size();i++)
		{
			params::goal_state[i] = state[i];
		}
	}


}

///////////This is kept around for testing purposes.
// params::integration_step=.002;
// params::stopping_type="iterations";
// params::stopping_check=100000;
// params::min_time_steps=20;
// params::max_time_steps=200;
// params::random_seed=time(NULL);
// params::sst_delta_near=.4;
// params::sst_delta_drain=.2;
// params::planner="sst";
// params::system="point";
// params::start_state=new double[2];
// params::goal_state=new double[2];
// params::start_state[0]=0;
// params::start_state[1]=0;
// params::goal_state[0]=9;
// params::goal_state[1]=9;
// params::goal_radius=.5;


// //Parameters for image output.
// params::tree_line_width=.5;
// params::solution_line_width=3;
// params::image_width=500;
// params::image_height=500;
// params::node_diameter=5;
// params::solution_node_diameter=4;

