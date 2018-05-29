
import _sst_module
import numpy as np
import time
import cv2
import os

import ConfigParser
import StringIO


from planners import SST, RRT


system_classes = {
    'car': _sst_module.Car,
    'cart_pole': _sst_module.CartPole,
    'pendulum': _sst_module.Pendulum,
    'point': _sst_module.Point,
    'rally_car': _sst_module.RallyCar,
    'two_link_acrobot': _sst_module.TwoLinkAcrobot,
}


def run_point_sst(config_path):

    config = ConfigParser.RawConfigParser()
    config.readfp(StringIO.StringIO('[root]\n' + open(config_path, 'r').read()))
    config = dict(config.items('root'))

    system = system_classes[config['system']]()

    start_state = np.array(map(float, config['start_state'].split()))
    goal_state = np.array(map(float, config['goal_state'].split()))
    goal_radius = float(config['goal_radius'])

    random_seed = int(config['random_seed'])

    print(config)
    if config['planner'] == 'sst':
        planner = SST(
            state_bounds=system.get_state_bounds(),
            control_bounds=system.get_control_bounds(),
            is_circular_topology=system.is_circular_topology(),
            start_state=start_state,
            goal_state=goal_state,
            goal_radius=goal_radius,
            random_seed=random_seed,
            sst_delta_near=float(config['sst_delta_near']),
            sst_delta_drain=float(config['sst_delta_drain'])
        )
    elif config['planner'] == 'rrt':
        planner = RRT(
            state_bounds=system.get_state_bounds(),
            control_bounds=system.get_control_bounds(),
            is_circular_topology=system.is_circular_topology(),
            start_state=start_state,
            goal_state=goal_state,
            goal_radius=goal_radius,
            random_seed=0,
        )
    else:
        raise Exception("Uknown planner")

    number_of_iterations = 300000

    min_time_steps = int(config['min_time_steps'])
    max_time_steps = int(config['max_time_steps'])
    integration_step = float(config['integration_step'])

    print("Starting the planner.")

    start_time = time.time()

    for iteration in range(number_of_iterations):
        planner.step(system, min_time_steps, max_time_steps, integration_step)
        if iteration % 1000 == 0:
            solution = planner.get_solution()

            if solution is None:
                solution_cost = None
            else:
                solution_cost = np.sum(solution[2])

            print("Time: %.2fs, Iterations: %d, Nodes: %d, Solution Quality: %s" %
                  (time.time() - start_time, iteration, planner.get_number_of_nodes(), solution_cost))

            im = planner.visualize_tree(system)
            cv2.imshow('tree', im)
            cv2.waitKey(1)

    path, controls, costs = planner.get_solution()
    solution_cost = np.sum(costs)

    print("Time: %.2fs, Iterations: %d, Nodes: %d, Solution Quality: %f" %
          (time.time() - start_time, number_of_iterations, planner.get_number_of_nodes(), solution_cost))

    im = planner.visualize_tree(system)
    cv2.imshow('tree', im)
    cv2.waitKey(-1)


if __name__ == '__main__':
    configs_path = os.path.join(os.path.dirname(__file__), "../input")
    run_point_sst(os.path.join(configs_path, 'rrt_pendulum.cfg'))
