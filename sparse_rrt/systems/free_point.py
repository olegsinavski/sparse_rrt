
import _sst_module
import numpy as np


class FreePoint(_sst_module.ISystem):
    MIN_X = -10
    MAX_X = 10
    MIN_Y = -10
    MAX_Y = 10

    MIN_V = 0
    MAX_V = 10
    MIN_THETA = -3.14
    MAX_THETA = 3.14

    def propagate(self, start_state, control, num_steps, integration_step):
        control_v = np.array([control[0] * np.cos(control[1]), control[0] * np.sin(control[1])])
        state = start_state + num_steps*integration_step*control_v
        state = np.clip(state, [self.MIN_X, self.MIN_Y], [self.MAX_X, self.MAX_Y])
        return state

    def get_state_bounds(self):
        return [(self.MIN_X, self.MAX_X),
                (self.MIN_Y, self.MAX_Y)]

    def get_control_bounds(self):
        return [(self.MIN_V, self.MAX_V),
                (self.MIN_THETA, self.MAX_THETA)]

    def is_circular_topology(self):
        return [False, False]

    def visualize_point(self, state):
        x = (state[0] - self.MIN_X) / (self.MAX_X - self.MIN_X)
        y = (state[1] - self.MIN_Y) / (self.MAX_Y - self.MIN_Y)
        return x, y
