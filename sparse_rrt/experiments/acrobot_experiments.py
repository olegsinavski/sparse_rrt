from sparse_rrt.experiments.experiment_utils import run_config

# experiments with Acrobot
base_acrobot_config = dict(
    start_state=[0., 0., 0., 0],
    goal_state=[3.14, 0., 0., 0.],
    goal_radius=2.0,
    random_seed=0,
    sst_delta_near=1.,
    sst_delta_drain=0.5,
    integration_step=0.02,
    min_time_steps=10,
    max_time_steps=50,
    number_of_iterations=300000,
    display_type='nodes'
)

# different configs for cpp and py implementations of the system
cpp_acrobot_config = dict(system='two_link_acrobot', **base_acrobot_config)
py_acrobot_config = dict(system='py_acrobot', **base_acrobot_config)


# different configs with different planners
rrt_acrobot_config = dict(planner='rrt', **cpp_acrobot_config)
sst_acrobot_config = dict(planner='sst', **cpp_acrobot_config)
rrt_py_acrobot_config = dict(planner='rrt', **py_acrobot_config)
sst_py_acrobot_config = dict(planner='sst', **py_acrobot_config)


if __name__ == '__main__':
    run_config(sst_py_acrobot_config)
