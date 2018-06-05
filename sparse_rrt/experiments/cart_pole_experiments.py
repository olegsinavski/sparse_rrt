from sparse_rrt.experiments.experiment_utils import run_config

# config for experiments with CartPole
cart_pole_config = dict(
    system='cart_pole',
    start_state=[-20, 0, 3.14, 0],
    goal_state=[20, 0, 3.14, 0],
    goal_radius=1.5,
    random_seed=0,
    sst_delta_near=2.,
    sst_delta_drain=1.2,
    integration_step=0.02,
    min_time_steps=10,
    max_time_steps=50,
    number_of_iterations=300000,
    display_type='tree'
)


# different configs with different planners
rrt_cart_pole_config = dict(planner='rrt', **cart_pole_config)
sst_cart_pole_config = dict(planner='sst', **cart_pole_config)


if __name__ == '__main__':
    run_config(sst_cart_pole_config)
