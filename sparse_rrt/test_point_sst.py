
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
        0: (1, None),
        100000: (4900, 2.486),
        200000: (5291, 2.072),
        300000: (5436, 1.996),
        400000: (5611, 1.988),
        'final': (5629, 1.988)
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

    expected_number_of_nodes, expected_solution_cost = expected_results['final']
    assert(planner.get_number_of_nodes() == expected_number_of_nodes)
    assert(abs(solution_cost - expected_solution_cost) < 1e-9)


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


def test_car_pose_rrt():
    system = _sst_module.CartPole()

    planner = _sst_module.RRTWrapper(
        state_bounds=system.get_state_bounds(),
        control_bounds=system.get_control_bounds(),
        is_circular_topology=system.is_circular_topology(),
        start_state=np.array([-20, 0, 3.14, 0]),
        goal_state=np.array([20, 0, 3.14, 0]),
        goal_radius=1.5,
        random_seed=0
    )

    min_time_steps = 10
    max_time_steps = 50
    integration_step = 0.02

    for iteration in range(10000):
        planner.step(system, min_time_steps, max_time_steps, integration_step)


def test_create_multiple_times():
    '''
    There used to be a crash during construction
    '''
    system = _sst_module.CartPole()
    planners = []
    for i in range(100):
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

        for iteration in range(100):
            planner.step(system, min_time_steps, max_time_steps, integration_step)
        planners.append(planner)


def test_custom_system_sst():

    class CustomSystem(_sst_module.ISystem):
        def propagate(self, start_state, control, num_steps, integration_step):
            return start_state + control*integration_step*num_steps

    planner = _sst_module.RRTWrapper(
        state_bounds=[(-10.0, 10.0), (-20.0, 20.0)],
        control_bounds=[(-1.0, 1.0), (-1.0, 1.0)],
        is_circular_topology=[False, False],
        start_state=np.array([0.2, 0.1]),
        goal_state=np.array([5., 5.]),
        goal_radius=1.5,
        random_seed=0
    )

    min_time_steps = 10
    max_time_steps = 50
    integration_step = 0.02

    system = CustomSystem()

    for iteration in range(10000):
        planner.step(system, min_time_steps, max_time_steps, integration_step)
        im = planner.visualize_tree(system)


if __name__ == '__main__':
    # st = time.time()
    # test_point_sst()
    # print(time.time() - st, 21.4076721668)
    #
    # test_car_pose_sst()
    # test_car_pose_rrt()
    # test_create_multiple_times()
    test_custom_system_sst()
    print('Passed all tests!')
