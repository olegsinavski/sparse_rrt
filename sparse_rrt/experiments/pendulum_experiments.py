from sparse_rrt.experiments.experiment_utils import run_config

# experiments with single-joint Pendulum
base_pendulum_config = dict(
    start_state=[0., 0.],
    goal_state=[1.57, 0.],
    goal_radius=0.1,
    random_seed=0,
    sst_delta_near=0.3,
    sst_delta_drain=0.1,
    integration_step=0.002,
    min_time_steps=20,
    max_time_steps=200,
    number_of_iterations=300000,
    display_type='tree'
)

# different configs for cpp and py implementations of the system
cpp_pendulum_config = dict(system='pendulum', **base_pendulum_config)
py_pendulum_config = dict(system='py_pendulum', **base_pendulum_config)


# different configs with different planners
rrt_pendulum_config = dict(planner='rrt', **cpp_pendulum_config)
sst_pendulum_config = dict(planner='sst', **cpp_pendulum_config)
rrt_py_pendulum_config = dict(planner='rrt', **py_pendulum_config)
sst_py_pendulum_config = dict(planner='sst', **py_pendulum_config)


if __name__ == '__main__':
    run_config(rrt_pendulum_config)
