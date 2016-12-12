

#include <boost/python.hpp>
#include <boost/format.hpp>
#include <iostream>

#include "utilities/numpy_boost_python.hpp"

#include "motion_planners/sst.hpp"
#include "motion_planners/rrt.hpp"
#include "systems/point.hpp"
#include "systems/car.hpp"
#include "systems/cart_pole.hpp"
#include "systems/pendulum.hpp"
#include "systems/rally_car.hpp"
#include "systems/two_link_acrobot.hpp"

#include "image_creation/planner_visualization.hpp"
#include "systems/distance_functions.h"

namespace py = boost::python;

void translate_exception(std::runtime_error const& e)
{
    // Use the Python 'C' API to set up an exception object
    PyErr_SetString(PyExc_RuntimeError, e.what());
}


class PlannerWrapper
{
public:

    void step(py::object system_object, int min_time_steps, int max_time_steps, double integration_step) {
        system_t& system = py::extract<system_t&>(system_object);
        planner->step(&system, min_time_steps, max_time_steps, integration_step);
    }

    void visualize_tree_wrapper(std::string const& file_name, py::object system_object) {

        int image_width = 500;
        int image_height = 500;
        double solution_node_diameter = 4;
        double solution_line_width = 3;
        double tree_line_width = 0.5;

        system_t& system = py::extract<system_t&>(system_object);

        std::vector<std::vector<double>> solution_path;
        std::vector<std::vector<double>> controls;
        std::vector<double> costs;
        planner->get_solution(solution_path, controls, costs);

        visualize_tree(
            file_name,
            planner->get_root(), solution_path, &system,
            planner->get_start_state(), planner->get_goal_state(),
            image_width, image_height, solution_node_diameter, solution_line_width, tree_line_width);
    }

    void visualize_nodes_wrapper(std::string const& file_name, py::object system_object) {
        int image_width = 500;
        int image_height = 500;
        double node_diameter = 5;
        double solution_node_diameter = 4;

        system_t& system = py::extract<system_t&>(system_object);

        std::vector<std::vector<double>> solution_path;
        std::vector<std::vector<double>> controls;
        std::vector<double> costs;
        planner->get_solution(solution_path, controls, costs);

        visualize_nodes(
            file_name,
            planner->get_root(), solution_path, &system,
            planner->get_start_state(), planner->get_goal_state(),
            image_width, image_height, node_diameter, solution_node_diameter);
    }

    py::object get_solution() {
        std::vector<std::vector<double>> solution_path;
        std::vector<std::vector<double>> controls;
        std::vector<double> costs;
        planner->get_solution(solution_path, controls, costs);

        if (controls.size() == 0) {
            return py::object();
        }

        numpy_boost<double, 2> controls_array({(long)controls.size(), (long)controls[0].size()});
        numpy_boost<double, 1> costs_array({(long)costs.size()});
        for (int i = 0; i < controls.size(); ++i) {
            for (int j = 0; j < controls[0].size(); ++j) {
                controls_array[i][j] = controls[i][j];
            }
            costs_array[i] = costs[i];
        }

        numpy_boost<double, 2> state_array({(long)solution_path.size(), (long)solution_path[0].size()});
        for (int i = 0; i < solution_path.size(); ++i) {
            for (int j = 0; j < solution_path[0].size(); ++j) {
                state_array[i][j] = solution_path[i][j];
            }
        }

        return py::make_tuple(state_array.py_object(), controls_array.py_object(), costs_array.py_object());
    }

    int get_number_of_nodes() {
        return this->planner->number_of_nodes;
    }

protected:
    std::unique_ptr<planner_t> planner;

};


class SSTWrapper : public PlannerWrapper{
public:
    SSTWrapper(
            const numpy_boost<double, 2> &state_bounds_array,
            const numpy_boost<double, 2> &control_bounds_array,
            const numpy_boost<bool, 1> &is_circular_topology_array,
            const numpy_boost<double, 1> &start_state,
            const numpy_boost<double, 1> &goal_state,
            double goal_radius,
            unsigned int random_seed,
            double sst_delta_near,
            double sst_delta_drain
    ) {

        if (state_bounds_array.shape()[0] != is_circular_topology_array.shape()[0]) {
            throw std::runtime_error("State and topology arrays have to be equal size");
        }

        if (state_bounds_array.shape()[0] != start_state.shape()[0]) {
            throw std::runtime_error("State bounds and start state arrays have to be equal size");
        }

        if (state_bounds_array.shape()[0] != goal_state.shape()[0]) {
            throw std::runtime_error("State bounds and goal state arrays have to be equal size");
        }

        typedef std::pair<double, double> bounds_t;
        std::vector<bounds_t> state_bounds;
        std::vector<bool> is_circular_topology;
        for (int i = 0; i < state_bounds_array.shape()[0]; i++) {
            state_bounds.push_back(bounds_t(state_bounds_array[i][0], state_bounds_array[i][1]));
            is_circular_topology.push_back(is_circular_topology_array[i]);
        }

        std::vector<bounds_t> control_bounds;
        for (int i = 0; i < control_bounds_array.shape()[0]; i++) {
            control_bounds.push_back(bounds_t(control_bounds_array[i][0], control_bounds_array[i][1]));
        }

        planner.reset(
                new sst_t(
                        state_bounds, control_bounds,
                        euclidian_distance(is_circular_topology),
                        random_seed,
                        sst_delta_near, sst_delta_drain)
        );

        planner->set_start_goal_state(&start_state[0], &goal_state[0], goal_radius);
        planner->setup_planning();

    }
};


class RRTWrapper : public PlannerWrapper{
public:
    RRTWrapper(
            const numpy_boost<double, 2> &state_bounds_array,
            const numpy_boost<double, 2> &control_bounds_array,
            const numpy_boost<bool, 1> &is_circular_topology_array,
            const numpy_boost<double, 1> &start_state,
            const numpy_boost<double, 1> &goal_state,
            double goal_radius,
            unsigned int random_seed
    ) {
        if (state_bounds_array.shape()[0] != is_circular_topology_array.shape()[0]) {
            throw std::runtime_error("State and topology arrays have to be equal size");
        }

        if (state_bounds_array.shape()[0] != start_state.shape()[0]) {
            throw std::runtime_error("State bounds and start state arrays have to be equal size");
        }

        if (state_bounds_array.shape()[0] != goal_state.shape()[0]) {
            throw std::runtime_error("State bounds and goal state arrays have to be equal size");
        }

        typedef std::pair<double, double> bounds_t;
        std::vector<bounds_t> state_bounds;
        std::vector<bool> is_circular_topology;
        for (int i = 0; i < state_bounds_array.shape()[0]; i++) {
            state_bounds.push_back(bounds_t(state_bounds_array[i][0], state_bounds_array[i][1]));
            is_circular_topology.push_back(is_circular_topology_array[i]);
        }

        std::vector<bounds_t> control_bounds;
        for (int i = 0; i < control_bounds_array.shape()[0]; i++) {
            control_bounds.push_back(bounds_t(control_bounds_array[i][0], control_bounds_array[i][1]));
        }

        planner.reset(
                new rrt_t(
                        state_bounds, control_bounds,
                        euclidian_distance(is_circular_topology),
                        random_seed)
        );

        planner->set_start_goal_state(&start_state[0], &goal_state[0], goal_radius);
        planner->setup_planning();

    }
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

    py::class_<PlannerWrapper, boost::noncopyable>(
        "PlannerWrapper", py::no_init)
            .def("step", &SSTWrapper::step)
            .def("visualize_tree", &SSTWrapper::visualize_tree_wrapper)
            .def("visualize_nodes", &SSTWrapper::visualize_nodes_wrapper)
            .def("get_solution", &SSTWrapper::get_solution)
            .def("get_number_of_nodes", &SSTWrapper::get_number_of_nodes)
    ;

    py::class_<SSTWrapper, py::bases<PlannerWrapper>, boost::noncopyable>(
        "SSTWrapper", py::init<
                const numpy_boost<double, 2>&,
                const numpy_boost<double, 2>&,
                const numpy_boost<bool, 1>&,
                const numpy_boost<double, 1>&,
                const numpy_boost<double, 1>&,
                double,
                unsigned int,
                double,
                double>((py::arg("state_bounds"), py::arg("control_bounds"), py::arg("is_circular_topology"),
                py::arg("start_state"), py::arg("goal_state"), py::arg("goal_radius"),
                py::arg("random_seed"), py::arg("sst_delta_near"), py::arg("sst_delta_drain"))))
    ;

    py::class_<RRTWrapper, py::bases<PlannerWrapper>, boost::noncopyable>(
            "RRTWrapper", py::init<
                    const numpy_boost<double, 2>&,
                    const numpy_boost<double, 2>&,
                    const numpy_boost<bool, 1>&,
                    const numpy_boost<double, 1>&,
                    const numpy_boost<double, 1>&,
                    double,
                    unsigned int>((py::arg("state_bounds"), py::arg("control_bounds"), py::arg("is_circular_topology"),
                    py::arg("start_state"), py::arg("goal_state"), py::arg("goal_radius"),
                    py::arg("random_seed"))))
    ;

    // this makes py::extract be able to extract a base class from systems
    py::class_<system_t, boost::noncopyable>("System", py::no_init)
        .def("get_state_bounds", &system_t::get_state_bounds)
        .def("get_control_bounds", &system_t::get_control_bounds)
        .def("is_circular_topology", &system_t::is_circular_topology)
    ;

    py::class_<car_t, py::bases<system_t>, boost::noncopyable>("Car", boost::python::init<>());
    py::class_<cart_pole_t, py::bases<system_t>, boost::noncopyable>("CartPole", boost::python::init<>());
    py::class_<pendulum_t, py::bases<system_t>, boost::noncopyable>("Pendulum", boost::python::init<>());
    py::class_<point_t, py::bases<system_t>, boost::noncopyable>("Point", boost::python::init<>());
    py::class_<rally_car_t, py::bases<system_t>, boost::noncopyable>("RallyCar", boost::python::init<>());
    py::class_<two_link_acrobot_t, py::bases<system_t>, boost::noncopyable>("TwoLinkAcrobot", boost::python::init<>());

}
