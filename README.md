# Python bindings for Sparse-RRT planner

This package is based on Sparse-RRT project https://bitbucket.org/pracsys/sparse_rrt/.
The main purpose of this work is to allow running Sparse-RRT planner in python environment. 
Here are the main contributions:
 - C++ experiments from original package are executable from Python while preserving speed, SVG visualizations and statistics
 - Controllable systems for the planners can be completely written in Python, while planners are executed in C++. This is not so slow as it sounds - the average slowdown is only 2-3x for system propagation.
 - C++ code of planners, utilities and systems underwent a lot of refactoring using moder C++11 features. Python bindings are done using pybind11
 - C++ implementation of planners, utilities and systems used to contain many memory leaks that prevent running planning multiple times. This was fixed in the current implementation.

The original codebase or SST planner is small enough to be easily understood and customized as opposed to OMPL SST implementation.

## INSTALLING
To install the package just run pip:

```
pip install sparse-rrt
```

## STANDARD EXPERIMENTS
To run a standard prepackaged experiment without visualization:
```python
from sparse_rrt.experiments import run_standard_experiment
run_standard_experiment('sst_car', visualization=False)
```

This will run SST planner for non-holonomic car system, implemented in C++ and print out statistics.
Right now, the reported statistics are the number of iterations 
executed, the number of nodes stored, and the solution length in 
seconds.

There are many more pre-packaged experiments. Run the following to get a list of them:
```python
from sparse_rrt.experiments import run_standard_experiment
for exp in available_standard_experiments():
    print(exp)
```

## VISUALIZATION

When running experiments, in addition to the terminal output,
visualization images can be displayed in the standalone window. This project inherits SVG-based visualization from original C++ codebase. There are few convenience functions
to display SVG images, to convert them to RGB numpy arrays or to display them in a window.
Visualization dependencies are not included in the package dependencies to ease installation. 
Here are the packages that you may need to install to run visualization:
 - `PySide` or `cairosvg` - to render SVG in python. `PySide` is considerably faster (to install run `pip install pyside` or `pip install cairosvg`)
 - `PySide` or `python-opencv` - to open windows and display numpy images (`show_image` function). To install opencv, you can follow <href>https://www.learnopencv.com/install-opencv3-on-ubuntu/
 - `svgwrite` - to write SVG files from python. This is needed only if you use visualization of Systems written in python (to install run `pip install svgwrite`)

The simplest option is to just install `PySide`. Here is how you can run standard experiment with visualization.
```python
from sparse_rrt.experiments import run_standard_experiment
run_standard_experiment('sst_car', visualization=True)
```

## CUSTOM EXPERIMENTS
Experiments are described using a python dictionary as a config. Examples of configs can be found in the `experiments` folder.
You can run custom experiment by creating your own config. For example, the following is going to run RRT planner with Point system with custom goal point:

```python
from sparse_rrt.experiments.experiment_utils import run_config
point_config = dict(
    system='point',
    planner='rrt',
    start_state=[9., 9.],
    goal_state=[-9., 9.],
    goal_radius=0.5,
    random_seed=100,
    integration_step=0.02,
    min_time_steps=2,
    max_time_steps=20,
    number_of_iterations=300000,
    display_type='tree'
)
run_config(point_config)
```

Experiment configs have a parameter `display_type` that can be either
 - `None`: no display
 - `'tree'`: image window shows the resulting tree of the motion
planner along with the solution path.
 - `'nodes'`: image windo shows the cost at each node in the tree. Darker nodes represent lower cost (better)
while lighter nodes denote higher cost.

## CUSTOM SYSTEMS
One of the main goals of this project is ability to plan for custom systems written in python.
In order to implement a custom system, you need to implement `sparse_rrt.systems.system_interface.ISystem` interface.
The main function to implement is `propagate` that is responsible for system dynamics and collision detection.
See docstrings for detailed documentation. A helper base class `sparse_rrt.systems.system_interface.BaseSystem` is provided for easy
creation of the most common systems. Here is the simplest example of a free point in 2D:
```python
import numpy as np
from sparse_rrt.systems.system_interface import BaseSystem

class FreePoint(BaseSystem):
    '''
    A simple system implementing a 2d point. It's controls include velocity and direction.
    '''
    MIN_X, MAX_X = -10, 10  # min and max X coordinate of a point
    MIN_Y, MAX_Y = -10, 10  # min and max Y coordinate of a point
    MIN_V, MAX_V = 0., 10.  # min and max control velocity
    MIN_THETA, MAX_THETA = -3.14, 3.14  # min and max control direction

    def propagate(self, start_state, control, num_steps, integration_step):
        '''
        Integrate system dynamics
        :param start_state: numpy array with the start state for the integration
        :param control: numpy array with constant controls to be applied during integration
        :param num_steps: number of steps to integrate
        :param integration_step: dt of integration
        :return: new state of the system
        '''
        control_v = np.array([control[0] * np.cos(control[1]), control[0] * np.sin(control[1])])
        trajectory = start_state + np.arange(num_steps)[:, None]*integration_step*control_v
        state = np.clip(trajectory[-1], [self.MIN_X, self.MIN_Y], [self.MAX_X, self.MAX_Y])
        return state

    def visualize_point(self, state):
        '''
        Project state space point to 2d visualization plane
        :param state: numpy array of the state point
        :return: x, y of visualization coordinates for this state point
        '''
        x = (state[0] - self.MIN_X) / (self.MAX_X - self.MIN_X)
        y = (state[1] - self.MIN_Y) / (self.MAX_Y - self.MIN_Y)
        return x, y

    def get_state_bounds(self):
        '''
        Return bounds for the state space
        :return: list of (min, max) bounds for each coordinate in the state space
        '''
        return [(self.MIN_X, self.MAX_X),
                (self.MIN_Y, self.MAX_Y)]

    def get_control_bounds(self):
        '''
        Return bounds for the control space
        :return: list of (min, max) bounds for each coordinate in the control space
        '''
        return [(self.MIN_V, self.MAX_V),
                (self.MIN_THETA, self.MAX_THETA)]

    def is_circular_topology(self):
        '''
        Indicate whether state system has planar or circular topology
        :return: boolean flag for each coordinate (False for planar topology)
        '''
        return [False, False]
```

In order to plan with your custom system, you can pass it in a `config['system']` field.

Alternatively, it is very easy to write a custom simulation completely from scratch:
```python
from sparse_rrt.planners import SST

# Create custom system
system = FreePoint()
# Create SST planner 
planner = SST(
    state_bounds=system.get_state_bounds(),
    control_bounds=system.get_control_bounds(),
    distance=system.distance_computer(),
    start_state=np.array([0., 0.]),
    goal_state=np.array([9., 9.]),
    goal_radius=0.5,
    random_seed=0,
    sst_delta_near=0.4,
    sst_delta_drain=0.2
)

# Run planning and print out solution is some statistics every few iterations.
for iteration in range(1000):
    planner.step(system, 2, 20, 0.1)
    if iteration % 100 == 0:
        solution = planner.get_solution()
        print("Solution: %s, Number of nodes: %s" % (planner.get_solution(), planner.get_number_of_nodes()))

```