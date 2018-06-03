#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pybind11/functional.h>

#include <iostream>

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


//namespace pybind11 {
//    template<typename T, uint8_t n_dim>
//    class safe_array: public array_t<T, pybind11::array::c_style> {
//    private:
//        typedef array_t<T, pybind11::array::c_style> ParentT;
//    public:
//        template <typename... Args>
//        safe_array(Args&&... args)
//            : ParentT(std::forward<Args>(args)...)
//        {
//            // Check if array is not empty and then check dimensions
//            if (ParentT::size() > 0 && ParentT::ndim() != n_dim) {
//                throw std::domain_error("Required array with " + std::to_string(n_dim) + " dimensions. Got " + std::to_string(ParentT::ndim()));
//            }
//        }
//    };
//};


namespace pybind11 {
    template <typename T>
    using safe_array = typename pybind11::array_t<T, pybind11::array::c_style>;
}

namespace py = pybind11;
using namespace pybind11::literals;


double global_dist(const double* point1, const double* point2, unsigned int state_dimensions){
    double result = 0;
    for (int i=0; i<state_dimensions; ++i) {
        result += (point1[i]-point2[i]) * (point1[i]-point2[i]);
    }
    return std::sqrt(result);
};



//std::function<double(const double*, const double*, unsigned int)> global_dist = [] (const double* point1, const double* point2, unsigned int state_dimensions) -> double {
//    double result = 0;
//    for (int i=0; i<state_dimensions; ++i) {
//        result += (point1[i]-point2[i]) * (point1[i]-point2[i]);
//    }
//    return std::sqrt(result);
//};


#include <assert.h>

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

py::safe_array<double> get_data(py::safe_array<double>& in_data) {
    //if (in_data.ndim() != 2) throw std::domain_error("error: ndim != 2");
    //in_data.mutable_data(0, 1);
//    double *p = in_data.mutable_data(0);
//    for (ssize_t i = 0; i < in_data.shape(0); i++) {
//        *p += 1;
//        ++p;
//    }

    throw std::domain_error("a");
    return in_data;
}


class PlannerWrapper
{
public:

    void step(system_interface& system, int min_time_steps, int max_time_steps, double integration_step) {
        planner->step(&system, min_time_steps, max_time_steps, integration_step);
    }

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
        for (int i = 0; i < controls.size(); ++i) {
            for (int j = 0; j < controls[0].size(); ++j) {
                controls_ref(i, j) = controls[i][j];
            }
            costs_ref(i) = costs[i];
        }

        py::safe_array<double> state_array({solution_path.size(), solution_path[0].size()});
        auto state_ref = state_array.mutable_unchecked<2>();
        for (int i = 0; i < solution_path.size(); ++i) {
            for (int j = 0; j < solution_path[0].size(); ++j) {
                state_ref(i, j) = solution_path[i][j];
            }
        }
        return py::cast(std::tuple<py::safe_array<double>, py::safe_array<double>, py::safe_array<double>>
            (state_array, controls_array, costs_array));
    }

    int get_number_of_nodes() {
        return this->planner->number_of_nodes;
    }

protected:
    std::unique_ptr<planner_t> planner;
};



class __attribute__ ((visibility ("hidden"))) SSTWrapper : public PlannerWrapper{
public:
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

        for (int i = 0; i < state_bounds_array.shape()[0]; i++) {
            state_bounds_v.push_back(bounds_t(state_bounds(i, 0), state_bounds(i, 1)));
        }

        std::vector<bounds_t> control_bounds_v;
        for (int i = 0; i < control_bounds_array.shape()[0]; i++) {
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
    py::object  _distance_computer_py;
};


class __attribute__ ((visibility ("hidden"))) RRTWrapper : public PlannerWrapper{
public:
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
        for (int i = 0; i < state_bounds_array.shape()[0]; i++) {
            state_bounds_v.push_back(bounds_t(state_bounds(i, 0), state_bounds(i, 1)));
        }

        std::vector<bounds_t> control_bounds_v;
        for (int i = 0; i < control_bounds_array.shape()[0]; i++) {
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
    py::object  _distance_computer_py;
};


class py_system_interface : public system_interface
{
public:

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

    std::string visualize_obstacles(int image_width, int image_height) const override
    {
    	PYBIND11_OVERLOAD(
            std::string, /* Return type */
            system_interface,      /* Parent class */
            visualize_obstacles,          /* Name of function in C++ (must match Python name) */
            image_width, image_height     /* Argument(s) */
        );
    }
};


PYBIND11_MODULE(_sst_module, m) {
   m.doc() = "Python wrapper for SST planners";

   //m.def("get_data", &get_data, py::arg("in_data").noconvert());

   py::class_<distance_t> distance_interface_var(m, "IDistance");
   py::class_<euclidean_distance, distance_t>(m, "EuclideanDistance");
   py::class_<two_link_acrobot_distance, distance_t>(m, "TwoLinkAcrobotDistance").def(py::init<>());

   m.def("euclidean_distance", &create_euclidean_distance, "is_circular_topology"_a.noconvert());

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
