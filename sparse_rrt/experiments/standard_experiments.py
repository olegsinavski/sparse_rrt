from sparse_rrt.experiments.acrobot_experiments import rrt_acrobot_config, sst_acrobot_config, rrt_py_acrobot_config, \
    sst_py_acrobot_config
from sparse_rrt.experiments.car_experiments import rrt_car_config, sst_car_config, rrt_py_car_config, sst_py_car_config
from sparse_rrt.experiments.cart_pole_experiments import rrt_cart_pole_config, sst_cart_pole_config
from sparse_rrt.experiments.experiment_utils import run_config
from sparse_rrt.experiments.pendulum_experiments import rrt_pendulum_config, sst_pendulum_config, \
    rrt_py_pendulum_config, sst_py_pendulum_config
from sparse_rrt.experiments.point_experiments import rrt_point_config, sst_point_config, rrt_py_point_config, \
    sst_py_point_config, rrt_free_point_config, sst_free_point_config
from sparse_rrt.experiments.rally_car_experiments import rrt_rally_car_config, sst_rally_car_config


standard_experiments = {
    'rrt_acrobot': rrt_acrobot_config,
    'sst_acrobot': sst_acrobot_config,
    'rrt_py_acrobot': rrt_py_acrobot_config,
    'sst_py_acrobot': sst_py_acrobot_config,
    'rrt_car': rrt_car_config,
    'sst_car': sst_car_config,
    'rrt_py_car': rrt_py_car_config,
    'sst_py_car': sst_py_car_config,
    'rrt_cart_pole': rrt_cart_pole_config,
    'sst_cart_pole': sst_cart_pole_config,
    'rrt_pendulum': rrt_pendulum_config,
    'sst_pendulum': sst_pendulum_config,
    'rrt_py_pendulum': rrt_py_pendulum_config,
    'sst_py_pendulum': sst_py_pendulum_config,
    'rrt_point': rrt_point_config,
    'sst_point': sst_point_config,
    'rrt_py_point': rrt_py_point_config,
    'sst_py_point': sst_py_point_config,
    'rrt_free_point': rrt_free_point_config,
    'sst_free_point': sst_free_point_config,
    'rrt_rally_car': rrt_rally_car_config,
    'sst_rally_car': sst_rally_car_config,
}


def run_standard_experiment(experiment_name, visualization=True):
    '''
    A helper to run an experiment from standard_experiments dict
    :param experiment_name: a key into standard_experiments
    :param visualization: whether to run visualization
    '''
    try:
        config = standard_experiments[experiment_name].copy()
    except KeyError:
        raise KeyError("There is no %s experiment. Available keys: %s" % (experiment_name, standard_experiments.keys()))
    if not visualization:
        config['display_type'] = None
    run_config(config)


def available_standard_experiments():
    '''
    Return a list of all available experiments
    '''
    return standard_experiments.keys()
