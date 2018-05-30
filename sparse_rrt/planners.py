import _sst_module
import cv2
import tempfile
import os
import shutil

import PySide.QtSvg
import PySide.QtGui

import numpy as np


def render_svg(svg_filepath):
    '''
    https://stackoverflow.com/questions/6589358/convert-svg-to-png-in-python/23093425#23093425
    '''
    r = PySide.QtSvg.QSvgRenderer(svg_filepath)
    image = PySide.QtGui.QImage(r.defaultSize().width(), r.defaultSize().height(), PySide.QtGui.QImage.Format_ARGB32)

    image.fill(PySide.QtGui.QColor(255, 255, 255, 255))

    r.render(PySide.QtGui.QPainter(image))
    width = image.width()
    height = image.height()

    ptr = image.bits()
    return np.array(ptr).reshape(height, width, 4)


def visualize_to_numpy(generator):
    d = tempfile.mkdtemp()
    name = os.path.join(d, 'tmp.svg')
    generator(name)
    assert(os.path.exists(name))
    im = render_svg(name)
    assert(im is not None)
    shutil.rmtree(d)
    return im


class SST(_sst_module.SSTWrapper):
    def visualize_tree(self, system):
        return visualize_to_numpy(lambda name: _sst_module.SSTWrapper.visualize_tree(self, name, system))

    def visualize_nodes(self, system):
        return visualize_to_numpy(lambda name: _sst_module.SSTWrapper.visualize_nodes(self, name, system))


class RRT(_sst_module.RRTWrapper):
    def visualize_tree(self, system):
        return visualize_to_numpy(lambda name: _sst_module.RRTWrapper.visualize_tree(self, name, system))

    def visualize_nodes(self, system):
        return visualize_to_numpy(lambda name: _sst_module.RRTWrapper.visualize_nodes(self, name, system))
