

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
#include "systems/distance_functions.h"

namespace py = boost::python;

void translate_exception(std::runtime_error const& e)
{
    // Use the Python 'C' API to set up an exception object
    PyErr_SetString(PyExc_RuntimeError, e.what());
}


class SSTWrapper
{
public:
    SSTWrapper(
        const numpy_boost<double, 2>& state_bounds_array,
        const numpy_boost<double, 2>& control_bounds_array,
        const numpy_boost<bool, 1>& is_circular_topology_array,
        unsigned int random_seed,
        double sst_delta_near,
        double sst_delta_drain
    )
    {
        std::cout << random_seed << sst_delta_near << sst_delta_drain << std::endl;
        if (state_bounds_array.shape()[0] != control_bounds_array.shape()[0]) {
            throw std::runtime_error("State and control bounds arrays have to be equal size");
        }

        if (state_bounds_array.shape()[0]  != is_circular_topology_array.shape()[0]) {
            throw std::runtime_error("State and topology arrays have to be equal size");
        }

        typedef std::pair<double, double> bounds_t;
        std::vector<bounds_t> state_bounds;
        std::vector<bounds_t> control_bounds;
        std::vector<bool> is_circular_topology;
        for (int i =0; i<state_bounds_array.shape()[0]; i++) {
            state_bounds.push_back(bounds_t(state_bounds_array[i][0], state_bounds_array[i][1]));
            control_bounds.push_back(bounds_t(control_bounds_array[i][0], control_bounds_array[i][1]));
            is_circular_topology.push_back(is_circular_topology_array[i]);
        }

        system_t* system = new point_t();
        planner.reset(
                new sst_t(
                        state_bounds, control_bounds,
                        euclidian_distance(is_circular_topology),
                        random_seed,
                        sst_delta_near, sst_delta_drain)
        );

    }

    void run() {
        // parameters
        std::string planner_name = "sst";
        std::string system_name = "point";

        double start_state[] = {0., 0.};
        double goal_state[] = {9., 9.};
        double goal_radius = 0.5;

        std::string stopping_type = "iterations";
        double stopping_check = 410000;

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

        unsigned int random_seed = 0;
        double sst_delta_near = 0.4;
        double sst_delta_drain = 0.2;

	    system_t* system = new point_t();

	    planner->set_start_goal_state(start_state, goal_state, goal_radius);
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


private:
    std::unique_ptr<sst_t> planner;

};


struct bounds_to_python
{
    static PyObject* convert(std::vector<std::pair<double, double> > const& bounds)
    {
        numpy_boost<double, 2> array_bounds({(long)bounds.size(), 2});
        for(int i = 0; i < bounds.size(); ++i) {
            array_bounds[i][0] = bounds[i].first;
            array_bounds[i][1] = bounds[i].second;
        }
        return boost::python::incref(array_bounds.py_ptr());
    }
};


struct flags_to_python
{
    static PyObject* convert(std::vector<bool> const& flags)
    {
        numpy_boost<bool, 1> array_flags({(long)flags.size()});
        for(int i = 0; i < flags.size(); ++i) {
            array_flags[i] = flags[i];
        }
        return boost::python::incref(array_flags.py_ptr());
    }
};


BOOST_PYTHON_MODULE(_sst_module)
{
    import_array();
    numpy_boost_python_register_type<uint8_t, 2>();
    numpy_boost_python_register_type<int32_t, 1>();
    numpy_boost_python_register_type<int32_t, 2>();
    numpy_boost_python_register_type<double, 1>();
    numpy_boost_python_register_type<double, 2>();
    numpy_boost_python_register_type<bool, 1>();
    numpy_boost_python_register_type<int, 2>();

    py::register_exception_translator<std::runtime_error>(&translate_exception);

    boost::python::to_python_converter<std::vector<std::pair<double, double>>, bounds_to_python>();
    boost::python::to_python_converter<std::vector<bool>, flags_to_python>();

    py::class_<SSTWrapper, boost::noncopyable>(
        "SSTWrapper", boost::python::init<
                    const numpy_boost<double, 2>&,
                    const numpy_boost<double, 2>&,
                    const numpy_boost<bool, 1>&,
                    unsigned int,
                    double,
                    double>())
            .def("run", &SSTWrapper::run)
    ;


    py::class_<point_t, boost::noncopyable>(
        "Point", boost::python::init<>())
            .def("get_state_bounds", &point_t::get_state_bounds)
            .def("get_control_bounds", &point_t::get_control_bounds)
            .def("is_circular_topology", &point_t::is_circular_topology)
    ;
}
