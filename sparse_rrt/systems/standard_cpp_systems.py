from sparse_rrt import _sst_module
import numpy as np


class Car(_sst_module.Car):
    def distance_computer(self):
        return _sst_module.euclidean_distance(np.array(self.is_circular_topology()))


class CartPole(_sst_module.CartPole):
    def distance_computer(self):
        return _sst_module.euclidean_distance(np.array(self.is_circular_topology()))


class Pendulum(_sst_module.Pendulum):
    def distance_computer(self):
        return _sst_module.euclidean_distance(np.array(self.is_circular_topology()))


class Point(_sst_module.Point):
    def distance_computer(self):
        return _sst_module.euclidean_distance(np.array(self.is_circular_topology()))


class RallyCar(_sst_module.RallyCar):
    def distance_computer(self):
        return _sst_module.euclidean_distance(np.array(self.is_circular_topology()))


class TwoLinkAcrobot(_sst_module.TwoLinkAcrobot):
    def distance_computer(self):
        return _sst_module.TwoLinkAcrobotDistance()
