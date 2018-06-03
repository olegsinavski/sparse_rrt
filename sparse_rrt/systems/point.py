
import numpy as np

from sparse_rrt.systems.system_interface import BaseSystem
from sparse_rrt.visualization import svg_rectangle


class Point(BaseSystem):
    '''
    A simple system implementing a 2d point. It's controls include velocity and direction.
    See function docs in BaseSystem definition
    '''
    MIN_X = -10
    MAX_X = 10
    MIN_Y = -10
    MAX_Y = 10

    MIN_V = 0
    MAX_V = 10
    MIN_THETA = -3.14
    MAX_THETA = 3.14

    def __init__(self, number_of_obstacles=5):
        BaseSystem.__init__(self)
        # define available obstacle rectangles
        available_obstacles = np.array((
            # Bottom Left X, Bottom Left Y, Top Right X, Top Right Y
            (1, -1.5, 5., 9.5),
            (-8., 4.25, -1, 5.75),
            (5., 3.5, 9, 4.5),
            (-8.5, -7.5, -3.5, -2.5),
            (5, -8.5, 9, -4.5)
        ))
        assert number_of_obstacles <= len(available_obstacles)
        # use only limited number of obstacles
        self._obstacles = available_obstacles[:number_of_obstacles]

    def propagate(self, start_state, control, num_steps, integration_step):
        control_v = np.array([control[0] * np.cos(control[1]), control[0] * np.sin(control[1])])
        trajectory = start_state + np.arange(num_steps)[:, None]*integration_step*control_v
        for (low_x, low_y, high_x, high_y) in self._obstacles:
            low_x_bound = trajectory[:, 0] >= low_x
            high_x_bound = trajectory[:, 0] <= high_x
            low_y_bound = trajectory[:, 1] >= low_y
            high_y_bound = trajectory[:, 1] <= high_y

            collisions = low_x_bound & high_x_bound & low_y_bound & high_y_bound
            if np.any(collisions):
                return None

        state = np.clip(trajectory[-1], [self.MIN_X, self.MIN_Y], [self.MAX_X, self.MAX_Y])
        return state

    def visualize_point(self, state):
        x = (state[0] - self.MIN_X) / (self.MAX_X - self.MIN_X)
        y = (state[1] - self.MIN_Y) / (self.MAX_Y - self.MIN_Y)
        return x, y

    def visualize_obstacles(self, image_width, image_height):
        '''
        Draw rectangles of obstacles
        '''
        output = ''
        for (low_x, low_y, high_x, high_y) in self._obstacles:
            x, y = self.visualize_point((low_x, low_y))
            output += svg_rectangle(
                (x * image_width, y * image_height),
                (image_width * (high_x - low_x) / float(self.MAX_X - self.MIN_X),
                 image_height * (high_y - low_y) / float(self.MAX_Y - self.MIN_Y)),
                (image_width, image_height),
                fill='red'
            )
        return output

    def get_state_bounds(self):
        return [(self.MIN_X, self.MAX_X),
                (self.MIN_Y, self.MAX_Y)]

    def get_control_bounds(self):
        return [(self.MIN_V, self.MAX_V),
                (self.MIN_THETA, self.MAX_THETA)]

    def is_circular_topology(self):
        return [False, False]
