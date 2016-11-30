
import _sst_module
import numpy as np


def test_point_sst():
    point = _sst_module.Point()

    sst = _sst_module.SSTWrapper(
        state_bounds=point.get_state_bounds(),
        control_bounds=point.get_control_bounds(),
        is_circular_topology=point.is_circular_topology(),
        start_state=np.array([0., 0.]),
        goal_state=np.array([9., 9.]),
        goal_radius=0.5,
        random_seed=0,
        sst_delta_near=0.4,
        sst_delta_drain=0.2
    )

    sst.run()

    sst.visualize(0, point)


if __name__ == '__main__':
    test_point_sst()