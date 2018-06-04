import standard_cpp_systems
from sparse_rrt.systems.acrobot import Acrobot
from sparse_rrt.systems.car import Car
from sparse_rrt.systems.pendulum import Pendulum
from sparse_rrt.systems.point import Point

# List of standard systems that come out of the box
_standard_system_classes = {
    'car': standard_cpp_systems.Car,
    'cart_pole': standard_cpp_systems.CartPole,
    'pendulum': standard_cpp_systems.Pendulum,
    'point': standard_cpp_systems.Point,
    'rally_car': standard_cpp_systems.RallyCar,
    'two_link_acrobot': standard_cpp_systems.TwoLinkAcrobot,
    'py_point': Point,
    'py_car': Car,
    'py_pendulum': Pendulum,
    'py_acrobot': Acrobot
}


def create_standard_system(system_name, *args, **kwargs):
    '''
    Create standard system by string identifier
    :param system_name: string, a name of the system from _standard_system_classes
    :param args: construction args of the system
    :param kwargs: construction kwargs of the system
    :return: A system that supports ISystem
    '''
    return _standard_system_classes[system_name](*args, **kwargs)
