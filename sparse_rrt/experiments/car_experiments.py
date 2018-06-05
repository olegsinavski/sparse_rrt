from sparse_rrt.experiments.experiment_utils import run_config

# config for experiments with Car
base_car_config = dict(
    start_state=[0., 0., 0.],
    goal_state=[9., 9., 0.],
    goal_radius=0.5,
    random_seed=0,
    sst_delta_near=0.6,
    sst_delta_drain=0.2,
    integration_step=0.002,
    min_time_steps=20,
    max_time_steps=200,
    debug_period=1000,
    number_of_iterations=300000,
    display_type='tree'
)

# different configs for cpp and py implementations of the system
cpp_car_config = dict(system='car', **base_car_config)
py_car_config = dict(system='py_car', **base_car_config)

# different configs with different planners
rrt_car_config = dict(planner='rrt', **cpp_car_config)
sst_car_config = dict(planner='sst', **cpp_car_config)
rrt_py_car_config = dict(planner='rrt', **py_car_config)
sst_py_car_config = dict(planner='sst', **py_car_config)


if __name__ == '__main__':
    run_config(rrt_car_config)
