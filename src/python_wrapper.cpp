/**
 * @file python_wrapper.cpp
 *
 * @copyright Software License Agreement (BSD License)
 * Original work Copyright 2017 Oleg Y. Sinyavskiy
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 *
 * Original authors: Oleg Y. Sinyavskiy
 *
 */

#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pybind11/functional.h>

#include <iostream>
#include <assert.h>

#include "systems/point.hpp"
#include "systems/car.hpp"
#include "systems/cart_pole.hpp"
#include "systems/pendulum.hpp"
#include "systems/rally_car.hpp"
#include "systems/two_link_acrobot.hpp"

#include "motion_planners/sst.hpp"
#include "motion_planners/rrt.hpp"

#include "image_creation/planner_visualization.hpp"
#include "systems/distance_functions.h"


namespace pybind11 {
    template <typename T>
    using safe_array = typename pybind11::array_t<T, pybind11::array::c_style>;
}

namespace py = pybind11;
using namespace pybind11::literals;


/**
 * @brief Python trampoline for abstract distance_t
 * @details Python trampoline for abstract distance_t to enable python classes override distance_t functions
 *
 */
class py_distance_interface : public distance_t
{
public:

	/**
	 * @copydoc distance_t::distance()
	 */
    double distance(const double* point1, const double* point2, unsigned int state_dimension) const override
    {
        // Copy cpp points to numpy arrays
        py::safe_array<double> point1_array{{state_dimension}};
        std::copy(point1, point1 + state_dimension, point1_array.mutable_data(0));

        py::safe_array<double> point2_array{{state_dimension}};
        std::copy(point2, point2 + state_dimension, point2_array.mutable_data(0));

        // Call python function
        py::gil_scoped_acquire gil;
        py::function overload = py::get_overload(static_cast<const distance_t *>(this), "distance");
        if (!overload) {
            pybind11::pybind11_fail("Tried to call pure virtual function distance");
            return false;
        }

        auto result = overload(point1_array, point2_array);
        // Extract double result
        return py::detail::cast_safe<double>(std::move(result));;
    }
};


/**
 * @brief Create cpp implementation of euclidean_distance
 * @details Create cpp implementation of euclidean_distance to use it from python
 *
 * @param is_circular_topology_array numpy array that indicates whether state coordinates have circular topology
 *
 * @return euclidean_distance object
 */
euclidean_distance create_euclidean_distance(
    const py::safe_array<bool> &is_circular_topology_array)
{
    auto is_circular_topology = is_circular_topology_array.unchecked<1>();
    std::vector<bool> is_circular_topology_v;
    for (int i = 0; i < is_circular_topology_array.shape()[0]; i++) {
        is_circular_topology_v.push_back(is_circular_topology(i));
    }
    return euclidean_distance(is_circular_topology_v);
}


/**
 * @brief Python wrapper for planner_t class
 * @details Python wrapper for planner_t class that handles numpy arguments and passes them to cpp functions
 *
 */
class PlannerWrapper
{
public:

    /**
	 * @copydoc planner_t::step()
	 */
    void step(system_interface& system, int min_time_steps, int max_time_steps, double integration_step) {
        planner->step(&system, min_time_steps, max_time_steps, integration_step);
    }

    /**
     * @brief Generate SVG visualization of the planning tree
     * @details Generate SVG visualization of the planning tree
     *
     * @param system an instance of the system to plan for
     * @param image_width Width of the drawing.
     * @param image_height Height of the drawing.
     * @param solution_node_diameter Diameter of a node.
     * @param solution_line_width Width of a planning solution path.
     * @param tree_line_width Width of the edges in the planning tree.
     *
     * @return string with SVG xml description
     */
    std::string visualize_tree_wrapper(
        system_interface& system,
        int image_width,
        int image_height,
        double solution_node_diameter,
        double solution_line_width,
        double tree_line_width)
    {
        std::vector<std::vector<double>> solution_path;
        std::vector<std::vector<double>> controls;
        std::vector<double> costs;
        planner->get_solution(solution_path, controls, costs);

        using namespace std::placeholders;
        std::string document_body = visualize_tree(
            planner->get_root(), solution_path,
            std::bind(&system_t::visualize_point, &system, _1, planner->get_state_dimension()),
            planner->get_start_state(), planner->get_goal_state(),
            image_width, image_height, solution_node_diameter, solution_line_width, tree_line_width);

        return std::move(document_body);
    }

    /**
     * @brief Generate SVG visualization of the nodes in the planning tree
     * @details Generate SVG visualization of the nodes in the planning tree
     *
     * @param system an instance of the system to plan for
     * @param image_width Width of the drawing.
     * @param image_height Height of the drawing.
     * @param node_diameter Diameter of nodes
     * @param solution_node_diameter Diameter of nodes that belong to the planning solution
     *
     * @return string with SVG xml description
     */
    std::string visualize_nodes_wrapper(
        system_interface& system,
        int image_width,
        int image_height,
        double node_diameter,
        double solution_node_diameter)
    {
        std::vector<std::vector<double>> solution_path;
        std::vector<std::vector<double>> controls;
        std::vector<double> costs;
        planner->get_solution(solution_path, controls, costs);

        using namespace std::placeholders;
        std::string document_body = visualize_nodes(
            planner->get_root(), solution_path,
            std::bind(&system_t::visualize_point, &system, _1, planner->get_state_dimension()),
            planner->get_start_state(),
            planner->get_goal_state(),
            image_width, image_height, node_diameter, solution_node_diameter);

        return std::move(document_body);
    }

    /**
	 * @copydoc planner_t::get_solution()
	 */
    py::object get_solution() {
        std::vector<std::vector<double>> solution_path;
        std::vector<std::vector<double>> controls;
        std::vector<double> costs;
        planner->get_solution(solution_path, controls, costs);

        if (controls.size() == 0) {
            return py::none();
        }

        py::safe_array<double> controls_array({controls.size(), controls[0].size()});
        py::safe_array<double> costs_array({costs.size()});
        auto controls_ref = controls_array.mutable_unchecked<2>();
        auto costs_ref = costs_array.mutable_unchecked<1>();
        for (unsigned int i = 0; i < controls.size(); ++i) {
            for (unsigned int j = 0; j < controls[0].size(); ++j) {
                controls_ref(i, j) = controls[i][j];
            }
            costs_ref(i) = costs[i];
        }

        py::safe_array<double> state_array({solution_path.size(), solution_path[0].size()});
        auto state_ref = state_array.mutable_unchecked<2>();
        for (unsigned int i = 0; i < solution_path.size(); ++i) {
            for (unsigned int j = 0; j < solution_path[0].size(); ++j) {
                state_ref(i, j) = solution_path[i][j];
            }
        }
        return py::cast(std::tuple<py::safe_array<double>, py::safe_array<double>, py::safe_array<double>>
            (state_array, controls_array, costs_array));
    }

    /**
	 * @copydoc planner_t::get_number_of_nodes()
	 */
    unsigned int get_number_of_nodes() {
        return this->planner->get_number_of_nodes();
    }

protected:
	/**
	 * @brief Created planner object
	 */
    std::unique_ptr<planner_t> planner;
};



/**
 * @brief Python wrapper for SST planner
 * @details Python wrapper for SST planner that handles numpy arguments and passes them to cpp functions
 *
 */
class __attribute__ ((visibility ("hidden"))) SSTWrapper : public PlannerWrapper{
public:

	/**
	 * @brief Python wrapper of SST planner Constructor
	 * @details Python wrapper of SST planner Constructor
	 *
	 * @param state_bounds_array numpy array (N x 2) with boundaries of the state space (min and max)
	 * @param control_bounds_array numpy array (N x 2) with boundaries of the control space (min and max)
	 * @param distance_computer_py Python wrapper of distance_t implementation
	 * @param start_state_array The start state (numpy array)
	 * @param goal_state_array The goal state  (numpy array)
	 * @param goal_radius The radial size of the goal region centered at in_goal.
	 * @param random_seed The seed for the random generator
	 * @param sst_delta_near Near distance threshold for SST
	 * @param sst_delta_drain Drain distance threshold for SST
	 */
    SSTWrapper(
            const py::safe_array<double> &state_bounds_array,
            const py::safe_array<double> &control_bounds_array,
            py::object distance_computer_py,
            const py::safe_array<double> &start_state_array,
            const py::safe_array<double> &goal_state_array,
            double goal_radius,
            unsigned int random_seed,
            double sst_delta_near,
            double sst_delta_drain
    )
        : _distance_computer_py(distance_computer_py)  // capture distance computer to avoid segfaults because we use a raw pointer from it
    {
        if (state_bounds_array.shape()[0] != start_state_array.shape()[0]) {
            throw std::domain_error("State bounds and start state arrays have to be equal size");
        }

        if (state_bounds_array.shape()[0] != goal_state_array.shape()[0]) {
            throw std::domain_error("State bounds and goal state arrays have to be equal size");
        }

        distance_t* distance_computer = distance_computer_py.cast<distance_t*>();

        auto state_bounds = state_bounds_array.unchecked<2>();
        auto control_bounds = control_bounds_array.unchecked<2>();
        auto start_state = start_state_array.unchecked<1>();
        auto goal_state = goal_state_array.unchecked<1>();

        typedef std::pair<double, double> bounds_t;
        std::vector<bounds_t> state_bounds_v;

        for (unsigned int i = 0; i < state_bounds_array.shape()[0]; i++) {
            state_bounds_v.push_back(bounds_t(state_bounds(i, 0), state_bounds(i, 1)));
        }

        std::vector<bounds_t> control_bounds_v;
        for (unsigned int i = 0; i < control_bounds_array.shape()[0]; i++) {
            control_bounds_v.push_back(bounds_t(control_bounds(i, 0), control_bounds(i, 1)));
        }

        std::function<double(const double*, const double*, unsigned int)>  distance_f =
            [distance_computer] (const double* p0, const double* p1, unsigned int dims) {
                return distance_computer->distance(p0, p1, dims);
            };

        planner.reset(
                new sst_t(
                        &start_state(0), &goal_state(0), goal_radius,
                        state_bounds_v, control_bounds_v,
                        distance_f,
                        random_seed,
                        sst_delta_near, sst_delta_drain)
        );
    }
private:

	/**
	 * @brief Captured distance computer python object to prevent its premature death
	 */
    py::object  _distance_computer_py;
};


class __attribute__ ((visibility ("hidden"))) RRTWrapper : public PlannerWrapper{
public:

	/**
	 * @brief Python wrapper of RRT planner constructor
	 * @details Python wrapper of RRT planner constructor
	 *
	 * @param state_bounds_array numpy array (N x 2) with boundaries of the state space (min and max)
	 * @param control_bounds_array numpy array (N x 2) with boundaries of the control space (min and max)
	 * @param distance_computer_py Python wrapper of distance_t implementation
	 * @param start_state_array The start state (numpy array)
	 * @param goal_state_array The goal state  (numpy array)
	 * @param goal_radius The radial size of the goal region centered at in_goal.
	 * @param random_seed The seed for the random generator
	 */
    RRTWrapper(
            const py::safe_array<double> &state_bounds_array,
            const py::safe_array<double> &control_bounds_array,
            py::object distance_computer_py,
            const py::safe_array<double> &start_state_array,
            const py::safe_array<double> &goal_state_array,
            double goal_radius,
            unsigned int random_seed
    ) : _distance_computer_py(distance_computer_py)
    {
        if (state_bounds_array.shape()[0] != start_state_array.shape()[0]) {
            throw std::runtime_error("State bounds and start state arrays have to be equal size");
        }

        if (state_bounds_array.shape()[0] != goal_state_array.shape()[0]) {
            throw std::runtime_error("State bounds and goal state arrays have to be equal size");
        }

        auto state_bounds = state_bounds_array.unchecked<2>();
        auto control_bounds = control_bounds_array.unchecked<2>();
        auto start_state = start_state_array.unchecked<1>();
        auto goal_state = goal_state_array.unchecked<1>();

        typedef std::pair<double, double> bounds_t;
        std::vector<bounds_t> state_bounds_v;
        for (unsigned int i = 0; i < state_bounds_array.shape()[0]; i++) {
            state_bounds_v.push_back(bounds_t(state_bounds(i, 0), state_bounds(i, 1)));
        }

        std::vector<bounds_t> control_bounds_v;
        for (unsigned int i = 0; i < control_bounds_array.shape()[0]; i++) {
            control_bounds_v.push_back(bounds_t(control_bounds(i, 0), control_bounds(i, 1)));
        }

        distance_t* distance_computer = distance_computer_py.cast<distance_t*>();
        std::function<double(const double*, const double*, unsigned int)>  distance_f =
            [distance_computer] (const double* p0, const double* p1, unsigned int dims) {
                return distance_computer->distance(p0, p1, dims);
            };

        planner.reset(
                new rrt_t(
                        &start_state(0), &goal_state(0), goal_radius,
                        state_bounds_v, control_bounds_v,
                        distance_f,
                        random_seed)
        );
    }
private:

	/**
	 * @brief Captured distance computer python object to prevent its premature death
	 */
    py::object  _distance_computer_py;
};


/**
 * @brief Python trampoline for system_interface distance_t
 * @details Python trampoline for system_interface distance_t to enable python classes override system_interface functions
 * and be able to create python systems
 *
 */
class py_system_interface : public system_interface
{
public:

	/**
	 * @copydoc system_interface::propagate()
	 */
    bool propagate(
        const double* start_state, unsigned int state_dimension,
        const double* control, unsigned int control_dimension,
        int num_steps,
        double* result_state, double integration_step) override
    {
        py::safe_array<double> start_state_array{{state_dimension}};
        std::copy(start_state, start_state + state_dimension, start_state_array.mutable_data(0));

        py::safe_array<double> control_array{{control_dimension}};
        std::copy(control, control + control_dimension, control_array.mutable_data(0));

        py::gil_scoped_acquire gil;
        py::function overload = py::get_overload(static_cast<const system_interface *>(this), "propagate");
        if (!overload) {
            pybind11::pybind11_fail("Tried to call pure virtual function propagate");
            return false;
        }

        auto result = overload(start_state_array, control_array, num_steps, integration_step);
        if (py::isinstance<py::none>(result)) {
            return false;
        } else {
            auto result_state_array = py::detail::cast_safe<py::safe_array<double>>(std::move(result));
            std::copy(result_state_array.data(0), result_state_array.data(0) + state_dimension, result_state);
            return true;
        }
    }

	/**
	 * @copydoc system_interface::visualize_point()
	 */
    std::tuple<double, double> visualize_point(const double* state, unsigned int state_dimension) const override {
        py::safe_array<double> state_array{{state_dimension}};
        std::copy(state, state + state_dimension, state_array.mutable_data(0));

        py::gil_scoped_acquire gil;
        py::function overload = py::get_overload(static_cast<const system_interface *>(this), "visualize_point");
        if (!overload) {
            pybind11::pybind11_fail("Tried to call pure virtual function visualize_point");
            return std::make_tuple(-1., -1.);
        }
        auto result = overload(state_array);
        return py::detail::cast_safe<std::tuple<double, double>>(std::move(result));
    }

	/**
	 * @copydoc system_interface::visualize_obstacles()
	 */
    std::string visualize_obstacles(int image_width, int image_height) const override
    {
    	PYBIND11_OVERLOAD(
            std::string,                /* Return type */
            system_interface,           /* Parent class */
            visualize_obstacles,        /* Name of function in C++ (must match Python name) */
            image_width, image_height   /* Argument(s) */
        );
    }
};


/**
 * @brief pybind module
 * @details pybind module for all planners, systems and interfaces
 *
 */
PYBIND11_MODULE(_sst_module, m) {
   m.doc() = "Python wrapper for SST planners";

   // Classes and interfaces for distance computation
   py::class_<distance_t, py_distance_interface> distance_interface_var(m, "IDistance");
   distance_interface_var
        .def(py::init<>());
   py::class_<euclidean_distance, distance_t>(m, "EuclideanDistance");
   py::class_<two_link_acrobot_distance, distance_t>(m, "TwoLinkAcrobotDistance").def(py::init<>());
   m.def("euclidean_distance", &create_euclidean_distance, "is_circular_topology"_a.noconvert());

   // Classes and interfaces for systems
   py::class_<system_interface, py_system_interface> system_interface_var(m, "ISystem");
   system_interface_var
        .def(py::init<>())
        .def("propagate", &system_interface::propagate)
        .def("visualize_point", &system_interface::visualize_point)
        .def("visualize_obstacles", &system_interface::visualize_obstacles);

   py::class_<system_t> system(m, "System", system_interface_var);
   system
        .def("get_state_bounds", &system_t::get_state_bounds)
        .def("get_control_bounds", &system_t::get_control_bounds)
        .def("is_circular_topology", &system_t::is_circular_topology)
   ;

   py::class_<car_t>(m, "Car", system).def(py::init<>());
   py::class_<cart_pole_t>(m, "CartPole", system).def(py::init<>());
   py::class_<pendulum_t>(m, "Pendulum", system).def(py::init<>());
   py::class_<point_t>(m, "Point", system)
       .def(py::init<int>(),
            "number_of_obstacles"_a=5
       );
   py::class_<rally_car_t>(m, "RallyCar", system).def(py::init<>());
   py::class_<two_link_acrobot_t>(m, "TwoLinkAcrobot", system).def(py::init<>());

   // Classes and interfaces for planners
   py::class_<PlannerWrapper> planner(m, "PlannerWrapper");
   planner
        .def("step", &PlannerWrapper::step)
        .def("visualize_tree", &PlannerWrapper::visualize_tree_wrapper,
            "system"_a,
            "image_width"_a=500,
            "image_height"_a=500,
            "solution_node_diameter"_a=4.,
            "solution_line_width"_a=3,
            "tree_line_width"_a=0.5
            )
        .def("visualize_nodes", &PlannerWrapper::visualize_nodes_wrapper,
            "system"_a,
            "image_width"_a=500,
            "image_height"_a=500,
            "node_diameter"_a=5,
            "solution_node_diameter"_a=4
            )
        .def("get_solution", &PlannerWrapper::get_solution)
        .def("get_number_of_nodes", &PlannerWrapper::get_number_of_nodes)
   ;

   py::class_<RRTWrapper>(m, "RRTWrapper", planner)
        .def(py::init<const py::safe_array<double>&,
                      const py::safe_array<double>&,
                      py::object,
                      const py::safe_array<double>&,
                      const py::safe_array<double>&,
                      double,
                      unsigned int>(),
            "state_bounds"_a,
            "control_bounds"_a,
            "distance"_a,
            "start_state"_a,
            "goal_state"_a,
            "goal_radius"_a,
            "random_seed"_a
        )
    ;

   py::class_<SSTWrapper>(m, "SSTWrapper", planner)
        .def(py::init<const py::safe_array<double>&,
                      const py::safe_array<double>&,
                      py::object,
                      const py::safe_array<double>&,
                      const py::safe_array<double>&,
                      double,
                      unsigned int,
                      double,
                      double>(),
            "state_bounds"_a,
            "control_bounds"_a,
            "distance"_a,
            "start_state"_a,
            "goal_state"_a,
            "goal_radius"_a,
            "random_seed"_a,
            "sst_delta_near"_a,
            "sst_delta_drain"_a
        )
   ;

}
