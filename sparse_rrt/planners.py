import _sst_module

from sparse_rrt.visualization import render_svg


def visualize_wrapper(parent_class):
    class VisualizeWrapper(parent_class):
        def visualize_tree(self, system, image_width=500, image_height=500):
            header, body, footer = parent_class.visualize_tree(
                self,
                system,
                image_width=image_width,
                image_height=image_height
            )
            return render_svg(header + body + footer)

        def visualize_nodes(self, system):
            return render_svg(parent_class.visualize_nodes(self, system))

    return VisualizeWrapper


class SST(visualize_wrapper(_sst_module.SSTWrapper)):
    pass


class RRT(visualize_wrapper(_sst_module.RRTWrapper)):
    pass
