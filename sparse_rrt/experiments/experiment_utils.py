import numpy as np
import time
import cv2

from sparse_rrt.planners import SST, RRT
from sparse_rrt.systems import create_standard_system


def run_config(config):
    '''
    A standard way to run an experiment config (in original cpp version configs were *.cfg files)
    :param config: Dict[String, object] with parameters of an experiment:
        integration_step: The simulation step (in seconds).  This is the resolution of the propagations.
        debug_period: The frequency for performing statistics gathering (0=don't perform statistics gathering)
        min_time_steps: The minimum number of simulation steps in a propagation.
        max_time_steps: The maximum number of simulation steps in a propagation.
        random_seed: The seed to the pseudo-random number generator
        sst_delta_near: The radius for BestNear (delta_n in the WAFR paper)
        sst_delta_drain: The radius for sparsification (delta_s in the WAFR paper)
        planner: The motion planner to use (string)
        system: The name of the system to plan for (or system object)
        start_state: The start state for the system
        goal_state: The goal state for the system
        goal_radius: The goal tolerance. (This is needed because in general, a dynamic system cannot reach a target state exactly.)
    '''
    if isinstance(config['system'], str):
        system = create_standard_system(config['system'])
    else:
        system = config['system']
    if config['planner'] == 'sst':
        planner = SST(
            state_bounds=system.get_state_bounds(),
            control_bounds=system.get_control_bounds(),
            distance=system.distance_computer(),
            start_state=config['start_state'],
            goal_state=config['goal_state'],
            goal_radius=config['goal_radius'],
            random_seed=config['random_seed'],
            sst_delta_near=float(config['sst_delta_near']),
            sst_delta_drain=float(config['sst_delta_drain'])
        )
    elif config['planner'] == 'rrt':
        planner = RRT(
            state_bounds=system.get_state_bounds(),
            control_bounds=system.get_control_bounds(),
            distance=system.distance_computer(),
            start_state=config['start_state'],
            goal_state=config['goal_state'],
            goal_radius=config['goal_radius'],
            random_seed=config['random_seed'],
        )
    else:
        raise Exception("Uknown planner")

    if 'number_of_iterations' not in config:
        config['number_of_iterations'] = int(1e6)

    if 'debug_period' not in config:
        config['debug_period'] = int(1e3)

    min_time_steps = int(config['min_time_steps'])
    max_time_steps = int(config['max_time_steps'])
    integration_step = float(config['integration_step'])

    run_planning_experiment(
        planner,
        system,
        config['number_of_iterations'],
        min_time_steps,
        max_time_steps,
        integration_step,
        config['debug_period']
    )


def run_planning_experiment(
    planner,
    system,
    number_of_iterations,
    min_time_steps,
    max_time_steps,
    integration_step,
    debug_period):
    '''
    Simple standard runner of a single planning experiment
    :param planner: planner instance
    :param system: system instance (ISystem)
    :param number_of_iterations: Int, how many iterations to run
    :param min_time_steps: Int, minimum number of steps to run the system during single node creation
    :param max_time_steps: Int, maximum number of steps to run the system during single node creation
    :param integration_step: Float, integration step to pass to the system during edge creation
    :param debug_period: A period when to print our debug information
    '''

    print("Starting the planner.")

    start_time = time.time()
    iteration_start_time = time.time()

    for iteration in range(number_of_iterations):
        planner.step(system, min_time_steps, max_time_steps, integration_step)
        if iteration % debug_period == 0:
            solution = planner.get_solution()

            if solution is None:
                solution_cost = None
            else:
                solution_cost = np.sum(solution[2])

            visualization_start_time = time.time()
            im = planner.visualize_tree(system)

            print("Time: %.2fs, Iterations: %d, Planning time: %.2fms Vis time: %.2fms Nodes: %d, Solution Quality: %s" %
                  (time.time() - start_time,
                   iteration,
                   1000*(time.time()-iteration_start_time)/debug_period,
                   1000*(time.time() - visualization_start_time),
                   planner.get_number_of_nodes(),
                   solution_cost))
            iteration_start_time = time.time()

            cv2.imshow('tree', im)
            cv2.waitKey(1)

    path, controls, costs = planner.get_solution()
    solution_cost = np.sum(costs)

    print("Time: %.2fs, Iterations: %d, Nodes: %d, Solution Quality: %f" %
          (time.time() - start_time, number_of_iterations, planner.get_number_of_nodes(), solution_cost))

    im = planner.visualize_tree(system)
    cv2.imshow('tree', im)
    cv2.waitKey(-1)
