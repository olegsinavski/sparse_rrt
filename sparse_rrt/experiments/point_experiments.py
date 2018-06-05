from sparse_rrt.systems import standard_cpp_systems

from sparse_rrt.experiments.experiment_utils import run_config


# experiments with a Point
base_point_config = dict(
    start_state=[0., 0.],
    goal_state=[9., 9.],
    goal_radius=0.5,
    random_seed=0,
    sst_delta_near=0.4,
    sst_delta_drain=0.2,
    integration_step=0.002,
    min_time_steps=20,
    max_time_steps=200,
    number_of_iterations=300000,
    display_type='tree'
)


# different configs for cpp and py implementations of the system
cpp_point_config = dict(system='point', **base_point_config)
py_point_config = dict(system='py_point', **base_point_config)
# a point without any obstacles
free_point_config = dict(system=standard_cpp_systems.Point(number_of_obstacles=0), **base_point_config)


# different configs with different planners
rrt_point_config = dict(planner='rrt', **cpp_point_config)
sst_point_config = dict(planner='sst', **cpp_point_config)
rrt_py_point_config = dict(planner='rrt', **py_point_config)
sst_py_point_config = dict(planner='sst', **py_point_config)

rrt_free_point_config = dict(planner='rrt', **free_point_config)
sst_free_point_config = dict(planner='sst', **free_point_config)

if __name__ == '__main__':
    run_config(rrt_point_config)
