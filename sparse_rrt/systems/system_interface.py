
import sparse_rrt._sst_module
from abc import abstractmethod


class ISystem(sparse_rrt._sst_module.ISystem):
    '''
    An interface for SST system defined in python. System is a controllable entity that is
    governed by certain dynamics under planner control.
    System live in a state space of "state_dimensions" dimensionality and accepts control of "control_dimensions" dimensionality.
    For example, single joint balancing pole would have state_dimensions==2 (angle and angular velocity) and
    control_dimensions==1 (torque in the root joint).
    Typically, system defines those dimensions, but in principle this can be decoupled and therefore its not part of
    the most basic interface.

    Integration of dynamics is done via "propagate" function. This function additionally is expected to perform
    collision checking and return True if propagation is valid and no collisions occured.

    Additionally, the system object can participate in the native SVG visualization framework via
    two functions: visualize_point and visualize_obstacles.
    TODO: separate visualization interface (the problem is that pybind is not particularly friendly with multiple inheritance)
    '''

    def __init__(self):
        '''
        Calling pybind constructor is necessary to avoid segfaults
        '''
        sparse_rrt._sst_module.ISystem.__init__(self)

    @abstractmethod
    def propagate(self, start_state, control, num_steps, integration_step):
        '''
        Integrate system dynamics under certain control for several steps.
        :param start_state: numpy vector of (state_dimensions,) shape that indicates a starting state for integration
        :param control: numpy vector of (control_dimensions,)
        :param num_steps: int of number of steps to integrate under the constant control
        :param integration_step: time step for integration
        :return: boolean indicating that integration is successful (e.g. it should return False if collision is detected)
        '''

    @abstractmethod
    def visualize_point(self, state):
        '''
        Project state space point of state_dimensions onto two dimensional plane for plane visualization
        :param state: numpy vector of (state_dimensions,) shape of a point in the state space
        :return: a tuple (x, y) of plane visualization coordinates for the state space point
        '''

    def visualize_obstacles(self, image_width, image_height):
        '''
        SVG visualization of obstacles pasted on top of the planner tree visualization
        :param image_width: width of the canvas
        :param image_height: height of the canvas
        :return: an SVG string that can be pasted after graph SVG description into SVG xml
        '''
        return ''


class BaseSystem(ISystem):
    '''
    It is typical that state and control space are defined by the system itself.
    This is a helper interface to document necessary descriptor functions
    '''
    @abstractmethod
    def get_state_bounds(self):
        '''
        Define state space bounds. Random planner is not going to sample points outside of this bounds
        :return: a list of tuples (min, max), where length of the list is equal to state_dimensions
            and each tuple indicate bounds of the particular state space coordinate
        '''

    @abstractmethod
    def get_control_bounds(self):
        '''
        Define control space bounds. Random planner is not going to sample control outside of this bounds
        :return: a list of tuples (min, max), where length of the list is equal to control_dimensions
            and each tuple indicate bounds of the particular control space coordinate
        '''

    @abstractmethod
    def is_circular_topology(self):
        '''
        Define a topology of the state space. Currently planners support planar and circular topology.
        This has to be indicated for every coordinate. For example, a mobile robot on a plane with a state
        space defined by its pose (x, y, direction) has to return [False, False, True] because x and y are planar
        and direction has circular topology

        :return: return a list of booleans, such that each boolean indicates whether a particular state space coordinate
            has circular topology (False for planar topology).
        '''

    def get_state_dimensions(self):
        '''
        A helper to determine state dimensions based on state bounds
        :return: integer == state_dimensions
        '''
        return len(self.get_state_bounds())

    def get_control_dimensions(self):
        '''
        A helper to determine control dimensions based on state bounds
        :return: integer == state_dimensions
        '''
        return len(self.get_control_dimensions())
