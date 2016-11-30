
import _sst_module


def test_point_sst():
    point = _sst_module.Point()

    # The radius for BestNear (delta_n in the WAFR paper)
    sst_delta_near=.4
    # The radius for sparsification (delta_s in the WAFR paper)
    st_delta_drain=.2

    sst = _sst_module.SSTWrapper(
        point.get_state_bounds(),
        point.get_control_bounds(),
        point.is_circular_topology(),
        0,
        sst_delta_near,
        st_delta_drain
    )

    sst.run()


if __name__ == '__main__':
    test_point_sst()