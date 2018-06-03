import _sst_module

from sparse_rrt.visualization import render_svg, svg_header, svg_footer


def visualize_wrapper(parent_class):
    class VisualizeWrapper(parent_class):
        def visualize_tree(self, system, image_width=500, image_height=500):
            body_string = parent_class.visualize_tree(
                self,
                system,
                image_width=image_width,
                image_height=image_height
            )
            body_string += system.visualize_obstacles(image_width, image_height)
            return render_svg(svg_header(width=image_width, height=image_height) + body_string + svg_footer())

        def visualize_nodes(self, system, image_width=500, image_height=500):
            body_string = parent_class.visualize_nodes(
                self,
                system,
                image_width=image_width,
                image_height=image_height
            )
            body_string += system.visualize_obstacles(image_width, image_height)
            return render_svg(svg_header(width=image_width, height=image_height) + body_string + svg_footer())

    return VisualizeWrapper


class SST(visualize_wrapper(_sst_module.SSTWrapper)):
    pass


class RRT(visualize_wrapper(_sst_module.RRTWrapper)):
    pass
