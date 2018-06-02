
import numpy as np

from sparse_rrt.systems.system_interface import BaseSystem


class Car(BaseSystem):
    '''
    Non-holonomic car
    '''
    MIN_X = -10
    MAX_X = 10
    MIN_Y = -10
    MAX_Y = 10
    MIN_ANGLE = -np.pi
    MAX_ANGLE = np.pi

    MIN_V = 0
    MAX_V = 1
    MIN_W = -0.5
    MAX_W = 0.5

    def propagate(self, start_state, control, num_steps, integration_step):
        '''
        Vectorized propagation of non-holonomic car
        '''
        # angle trajectory
        angles = start_state[2] + np.arange(1, num_steps+1)*(integration_step*control[1])
        # x and y projections
        coss = np.cos(angles)*(integration_step*control[0])
        sins = np.sin(angles)*(integration_step*control[0])
        state = start_state
        # integrate
        state[0] = start_state[0] + np.sum(coss)
        state[1] = start_state[1] + np.sum(sins)
        state[2] = angles[-1]
        state = np.clip(state, [self.MIN_X, self.MIN_Y, self.MIN_ANGLE], [self.MAX_X, self.MAX_Y, self.MAX_ANGLE])
        return state

    def visualize_point(self, state):
        x = (state[0] - self.MIN_X) / (self.MAX_X - self.MIN_X)
        y = (state[1] - self.MIN_Y) / (self.MAX_Y - self.MIN_Y)
        return x, y

    def get_state_bounds(self):
        return [(self.MIN_X, self.MAX_X),
                (self.MIN_Y, self.MAX_Y),
                (self.MIN_ANGLE, self.MAX_ANGLE)]

    def get_control_bounds(self):
        return [(self.MIN_V, self.MAX_V),
                (self.MIN_W, self.MAX_W)]

    def is_circular_topology(self):
        return [False, False, True]
