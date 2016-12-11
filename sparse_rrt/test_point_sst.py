
import _sst_module
import numpy as np
import time


def test_point_sst():
    system = _sst_module.Point()

    planner = _sst_module.SSTWrapper(
        state_bounds=system.get_state_bounds(),
        control_bounds=system.get_control_bounds(),
        is_circular_topology=system.is_circular_topology(),
        start_state=np.array([0., 0.]),
        goal_state=np.array([9., 9.]),
        goal_radius=0.5,
        random_seed=0,
        sst_delta_near=0.4,
        sst_delta_drain=0.2
    )

    number_of_iterations = 410000

    min_time_steps = 20
    max_time_steps = 200
    integration_step = 0.002

    print("Starting the planner.")

    start_time = time.time()

    expected_results = {
        0: (2, None),
        100000: (5037, 2.53),
        200000: (5260, 2.152),
        300000: (5510, 2.076),
        400000: (5694, 2.062),
    }

    for iteration in range(number_of_iterations):
        planner.step(system, min_time_steps, max_time_steps, integration_step)
        if iteration % 100000 == 0:
            solution = planner.get_solution()

            expected_number_of_nodes, expected_solution_cost = expected_results[iteration]
            assert(expected_number_of_nodes == planner.get_number_of_nodes())

            if solution is None:
                solution_cost = None
                assert(expected_solution_cost is None)
            else:
                solution_cost = np.sum(solution[2])
                assert(abs(solution_cost - expected_solution_cost) < 1e-9)

            print("Time: %.2fs, Iterations: %d, Nodes: %d, Solution Quality: %s" %
                  (time.time() - start_time, iteration, planner.get_number_of_nodes(), solution_cost))

    path, controls, costs = planner.get_solution()
    solution_cost = np.sum(costs)

    print("Time: %.2fs, Iterations: %d, Nodes: %d, Solution Quality: %f" %
          (time.time() - start_time, number_of_iterations, planner.get_number_of_nodes(), solution_cost))

    assert(planner.get_number_of_nodes() == 5716)
    assert(abs(solution_cost - 2.062) < 1e-9)


def test_car_pose_sst():
    system = _sst_module.CartPole()

    planner = _sst_module.SSTWrapper(
        state_bounds=system.get_state_bounds(),
        control_bounds=system.get_control_bounds(),
        is_circular_topology=system.is_circular_topology(),
        start_state=np.array([-20, 0, 3.14, 0]),
        goal_state=np.array([20, 0, 3.14, 0]),
        goal_radius=1.5,
        random_seed=0,
        sst_delta_near=2.,
        sst_delta_drain=1.2
    )

    min_time_steps = 10
    max_time_steps = 50
    integration_step = 0.02

    for iteration in range(10000):
        planner.step(system, min_time_steps, max_time_steps, integration_step)


if __name__ == '__main__':
    test_point_sst()
    test_car_pose_sst()
    print('Passed all tests!')
