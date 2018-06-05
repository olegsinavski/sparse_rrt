
import numpy as np

from sparse_rrt.systems.system_interface import BaseSystem


class Pendulum(BaseSystem):
    '''
    Single joint pendulum.
    '''
    MIN_ANGLE, MAX_ANGLE = -np.pi, np.pi
    MIN_W, MAX_W = -7., 7

    MIN_TORQUE, MAX_TORQUE = -1., 1.

    LENGTH = 1.
    MASS = 1.
    DAMPING = .05

    def propagate(self, start_state, control, num_steps, integration_step):
        gravity_coeff = self.MASS*9.81*self.LENGTH*0.5
        integration_coeff = 3. / (self.MASS*self.LENGTH*self.LENGTH)*integration_step
        state = start_state
        for i in range(num_steps):
            state[0] += integration_step * state[1]
            state[1] += integration_coeff * (control[0] - gravity_coeff*np.cos(state[0]) - self.DAMPING*state[1])

            if state[0] < -np.pi:
                state[0] += 2*np.pi
            elif state[0] > np.pi:
                state[0] -= 2 * np.pi
            state = np.clip(state, [self.MIN_ANGLE, self.MIN_W], [self.MAX_ANGLE, self.MAX_W])
        return state

    def visualize_point(self, state):
        x = (state[0] + np.pi) / (2*np.pi)
        y = (state[1] - self.MIN_W) / (self.MAX_W - self.MIN_W)
        return x, y

    def get_state_bounds(self):
        return [(self.MIN_ANGLE, self.MAX_ANGLE),
                (self.MIN_W, self.MAX_W)]

    def get_control_bounds(self):
        return [(self.MIN_TORQUE, self.MAX_TORQUE)]

    def is_circular_topology(self):
        return [True, False]
