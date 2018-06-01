
import _sst_module
import numpy as np
import time
import cv2
import os

import ConfigParser
import StringIO


from planners import SST, RRT
from sparse_rrt.systems.free_point import FreePoint


system_classes = {
    'car': _sst_module.Car,
    'cart_pole': _sst_module.CartPole,
    'pendulum': _sst_module.Pendulum,
    'point': _sst_module.Point,
    'free_point': lambda: _sst_module.Point(number_of_obstacles=0),
    'rally_car': _sst_module.RallyCar,
    'two_link_acrobot': _sst_module.TwoLinkAcrobot,
    'py_free_point': FreePoint
}


def load_standard_config(config_name):
    config_path = os.path.join(os.path.dirname(__file__), "../input")
    config_path = os.path.join(config_path, config_name+'.cfg')
    config = ConfigParser.RawConfigParser()
    config.readfp(StringIO.StringIO('[root]\n' + open(config_path, 'r').read()))
    config = dict(config.items('root'))

    config['start_state'] = np.array(map(float, config['start_state'].split()))
    config['goal_state'] = np.array(map(float, config['goal_state'].split()))
    config['goal_radius'] = float(config['goal_radius'])
    config['random_seed'] = int(config['random_seed'])

    return config


def run_config(config):
    system = system_classes[config['system']]()

    print(config)
    if config['planner'] == 'sst':
        planner = SST(
            state_bounds=system.get_state_bounds(),
            control_bounds=system.get_control_bounds(),
            is_circular_topology=system.is_circular_topology(),
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
            is_circular_topology=system.is_circular_topology(),
            start_state=config['start_state'],
            goal_state=config['goal_state'],
            goal_radius=config['goal_radius'],
            random_seed=config['random_seed'],
        )
    else:
        raise Exception("Uknown planner")

    number_of_iterations = 300000

    min_time_steps = int(config['min_time_steps'])
    max_time_steps = int(config['max_time_steps'])
    integration_step = float(config['integration_step'])

    run_planning_experiment(
        planner,
        system,
        number_of_iterations,
        min_time_steps,
        max_time_steps,
        integration_step
    )


def run_planning_experiment(
        planner, system,
        number_of_iterations,
        min_time_steps, max_time_steps, integration_step):

    print("Starting the planner.")

    start_time = time.time()
    iteration_start_time = time.time()

    debug_period = 1000
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


python_configs = {
    'py_free_point_sst': dict(
        system='py_free_point',
        planner='sst',
        start_state=np.array([0., 0.]),
        goal_state=np.array([9., 9.]),
        goal_radius=0.5,
        random_seed=0,
        sst_delta_near=0.4,
        sst_delta_drain=0.2,
        integration_step=0.002,
        min_time_steps=20,
        max_time_steps=200,
        number_of_iterations=300000)
}


def find_config(name):
    if name in python_configs:
        return python_configs[name]
    return load_standard_config(name)


if __name__ == '__main__':
    run_config(find_config('py_free_point_sst'))

'''
Took 0.0482540130615 to convert
Time: 14.97s, Iterations: 103000, Planning time: 0.15ms Vis time: 80.99ms Nodes: 6524, Solution Quality: 1.49
Took 0.0517320632935 to convert
Time: 15.12s, Iterations: 104000, Planning time: 0.15ms Vis time: 84.69ms Nodes: 6548, Solution Quality: 1.49
Took 0.05157995224 to convert
Time: 15.28s, Iterations: 105000, Planning time: 0.17ms Vis time: 86.73ms Nodes: 6560, Solution Quality: 1.49
Took 0.0501899719238 to convert
Time: 15.44s, Iterations: 106000, Planning time: 0.16ms Vis time: 83.15ms Nodes: 6573, Solution Quality: 1.49
Took 0.0520739555359 to convert
Time: 15.60s, Iterations: 107000, Planning time: 0.16ms Vis time: 86.87ms Nodes: 6576, Solution Quality: 1.49
Took 0.0505089759827 to convert
Time: 15.77s, Iterations: 108000, Planning time: 0.17ms Vis time: 86.09ms Nodes: 6567, Solution Quality: 1.48
Took 0.0507559776306 to convert
Time: 15.91s, Iterations: 109000, Planning time: 0.14ms Vis time: 83.39ms Nodes: 6576, Solution Quality: 1.48
Took 0.0552539825439 to convert
Time: 16.09s, Iterations: 110000, Planning time: 0.18ms Vis time: 92.70ms Nodes: 6585, Solution Quality: 1.48
Took 0.048957824707 to convert
Time: 16.24s, Iterations: 111000, Planning time: 0.15ms Vis time: 82.75ms Nodes: 6591, Solution Quality: 1.48
Took 0.0516459941864 to convert
Time: 16.39s, Iterations: 112000, Planning time: 0.15ms Vis time: 85.87ms Nodes: 6612, Solution Quality: 1.48
'''