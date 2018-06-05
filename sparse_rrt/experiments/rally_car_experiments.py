from sparse_rrt.experiments.experiment_utils import run_config


# experiments with a 6 dimensional rally car
rally_car_config = dict(
    system='rally_car',
    start_state=[18., -30, 0, 16, 1.57, 0, 0, 0],
    goal_state=[-18., -30, 0, -16, -1.57, 0, 0, 0],
    goal_radius=5,
    random_seed=0,
    sst_delta_near=.5,
    sst_delta_drain=.3,
    integration_step=0.002,
    min_time_steps=20,
    max_time_steps=200,
    number_of_iterations=300000,
    display_type='tree'
)

# different configs with different planners
rrt_rally_car_config = dict(planner='rrt', **rally_car_config)
sst_rally_car_config = dict(planner='sst', **rally_car_config)


if __name__ == '__main__':
    run_config(sst_rally_car_config)
