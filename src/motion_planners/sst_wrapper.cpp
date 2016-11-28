

#include <boost/python.hpp>
#include <boost/format.hpp>
#include <iostream>
#include <functional>

#include "utilities/numpy_boost_python.hpp"

#include "motion_planners/sst.hpp"
#include "systems/point.hpp"
#include "utilities/condition_check.hpp"
#include "utilities/random.hpp"
#include "utilities/timer.hpp"
#include "image_creation/planner_visualization.hpp"

namespace py = boost::python;

void translate_exception(std::runtime_error const& e)
{
    // Use the Python 'C' API to set up an exception object
    PyErr_SetString(PyExc_RuntimeError, e.what());
}


class SSTWrapper
{
public:
    SSTWrapper()
    {

    }

    void run() {
        // parameters
        std::string planner_name = "sst";
        std::string system_name = "point";

        int random_seed = 0;
        double start_state[] = {0., 0.};
        double goal_state[] = {9., 9.};
        double goal_radius = 0.5;

        // The radius for BestNear (delta_n in the WAFR paper)
        double sst_delta_near=.4;
        // The radius for sparsification (delta_s in the WAFR paper)
        double sst_delta_drain=.2;

        std::string stopping_type = "time";
        double stopping_check = 15;

        std::string stats_type = "time";
        double stats_check_value = 0;

        bool intermediate_visualization = false;

        int image_width = 500;
        int image_height = 500;
        double node_diameter = 5;
        double solution_node_diameter = 4;
        double solution_line_width = 3;
        double tree_line_width = 0.5;

        int min_time_steps = 20;
        int max_time_steps = 200;
        double integration_step = 0.002;

        init_random(random_seed);

	    system_t* system = new point_t();
	    planner_t* planner = new sst_t(system->get_state_bounds(), system->get_control_bounds(),
                                       std::bind( &system_t::distance, system, std::placeholders::_1, std::placeholders::_2),
                                       sst_delta_near, sst_delta_drain);

	    planner->set_start_state(start_state);
	    planner->set_goal_state(goal_state, goal_radius);
	    planner->setup_planning();

	    condition_check_t checker(stopping_type, stopping_check);
	    condition_check_t* stats_check=NULL;
	    if(stats_check_value!=0)
        {
            stats_check = new condition_check_t(stats_type, stats_check_value);
        }

        checker.reset();
        std::cout<<"Starting the planner: "<<planner_name<<" for the system: "<<system_name<<std::endl;

        int count = 0;
        bool execution_done = false;
        bool stats_print = false;
        while(true)
        {
            do
            {
                planner->step(system, min_time_steps, max_time_steps, integration_step);
                execution_done = checker.check();
                if (stats_check != NULL) {
                    stats_print = stats_check->check();
                }
            }
            while(!execution_done && !stats_print);

            std::vector<std::pair<double*,double> > controls;
            planner->get_solution(controls);
            double solution_cost = 0;
            for(unsigned i=0;i<controls.size();i++)
            {
                solution_cost+=controls[i].second;
            }
            std::cout<<"Time: "<<checker.time()<<" Iterations: "<<checker.iterations()<<" Nodes: "<<planner->number_of_nodes<<" Solution Quality: " <<solution_cost<<std::endl ;
            stats_print = false;
            if(intermediate_visualization || execution_done)
            {
                visualize_tree(
                    planner->get_root(), planner->get_last_solution_path(), system, start_state, goal_state,
                    count, image_width, image_height, solution_node_diameter, solution_line_width, tree_line_width);
                visualize_nodes(
                    planner->get_root(), planner->get_last_solution_path(), system, start_state, goal_state,
                    count, image_width, image_height, node_diameter, solution_node_diameter);
                count++;
            }
            if (stats_check != NULL) {
                stats_check->reset();
            }

            if (execution_done)
            {
                break;
            }
        }
        std::cout<<"Done planning."<<std::endl;


    }

};


BOOST_PYTHON_MODULE(_sst_module)
{
    import_array();
//    numpy_boost_python_register_type<uint8_t, 2>();
//    numpy_boost_python_register_type<int32_t, 1>();
//    numpy_boost_python_register_type<int32_t, 2>();
//    numpy_boost_python_register_type<double, 1>();
//    numpy_boost_python_register_type<double, 2>();
//    numpy_boost_python_register_type<int, 2>();

    //py::register_exception_translator<std::runtime_error>(&translate_exception);

    py::class_<SSTWrapper, boost::noncopyable>(
        "SSTWrapper", boost::python::init<>())
            .def("run", &SSTWrapper::run)
    ;
}
