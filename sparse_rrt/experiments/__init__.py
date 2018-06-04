from sparse_rrt.experiments.car_experiments import rrt_car_config, sst_car_config, rrt_py_car_config, sst_py_car_config
from sparse_rrt.experiments.cart_pole_experiments import rrt_cart_pole_config, sst_cart_pole_config
from sparse_rrt.experiments.point_experiments import rrt_point_config, sst_point_config, rrt_py_point_config, \
    sst_py_point_config, rrt_free_point_config, sst_free_point_config
from sparse_rrt.experiments.rally_car_experiments import rrt_rally_car_config, sst_rally_car_config

standard_experiments = {
    'rrt_point': rrt_point_config,
    'sst_point': sst_point_config,
    'rrt_py_point': rrt_py_point_config,
    'sst_py_point': sst_py_point_config,
    'rrt_free_point': rrt_free_point_config,
    'sst_free_point': sst_free_point_config,
    'rrt_rally_car': rrt_rally_car_config,
    'sst_rally_car': sst_rally_car_config,
    'rrt_car': rrt_car_config,
    'sst_car': sst_car_config,
    'rrt_py_car': rrt_py_car_config,
    'sst_py_car': sst_py_car_config,
    'rrt_cart_pole': rrt_cart_pole_config,
    'sst_cart_pole': sst_cart_pole_config,
}
