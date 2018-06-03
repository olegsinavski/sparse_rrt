
import time
import numpy as np


def svg_header(width, height):
    return '''<?xml version="1.0" standalone="no" ?>
<!DOCTYPE svg PUBLIC "-//W3C//DTD SVG 1.1//EN" "http://www.w3.org/Graphics/SVG/1.1/DTD/svg11.dtd">
<svg width="%dpx" height="%dpx" xmlns="http://www.w3.org/2000/svg" version="1.1" >

''' % (width, height)


def svg_footer():
    return '\n</svg>'


def render_svg_pyside(svg_string):
    '''
    Render svg file into numpy array using pyside (the fastest implementation)
    https://stackoverflow.com/questions/6589358/convert-svg-to-png-in-python/23093425#23093425
    '''
    import PySide.QtSvg
    import PySide.QtGui
    r = PySide.QtSvg.QSvgRenderer(PySide.QtCore.QXmlStreamReader(PySide.QtCore.QByteArray(svg_string)))
    image = PySide.QtGui.QImage(r.defaultSize().width(), r.defaultSize().height(), PySide.QtGui.QImage.Format_ARGB32)

    image.fill(PySide.QtGui.QColor(255, 255, 255, 255))

    r.render(PySide.QtGui.QPainter(image))
    width = image.width()
    height = image.height()
    ptr = image.bits()
    return np.array(ptr).reshape(height, width, 4)


def render_svg_cairosvg(svg_string):
    '''
    Render svg file into numpy array using cairosvg (slower than pyside)
    Tested with cairosvg==1.0.22
    https://stackoverflow.com/questions/6589358/convert-svg-to-png-in-python/23093425#23093425
    '''
    from cairosvg import surface
    try:
        import cairocffi as cairo
    # OSError means cairocffi is installed,
    # but could not load a cairo dynamic library.
    # pycairo may still be available with a statically-linked cairo.
    except (ImportError, OSError):
        import cairo  # pycairo

    class BinarySurface(surface.Surface):
        """An implementation of the cairosvg surface that doesn't write to png, but just dumps data into RGB numpy array"""
        device_units_per_user_units = 1

        def _create_surface(self, width, height):
            """Create and return ``(cairo_surface, width, height)``."""
            width = int(width)
            height = int(height)
            cairo_surface = cairo.ImageSurface(cairo.FORMAT_ARGB32, width, height)
            return cairo_surface, width, height

        def finish(self):
            """
            Copy cairo surface to numpy array and invert black color to white
            """
            cairo_array = np.frombuffer(self.cairo.get_data(), np.uint8).reshape(
                self.cairo.get_height(), self.cairo.get_width(), 4)
            img = np.ones((self.cairo.get_height(), self.cairo.get_width(), 3), np.uint8) * 255
            non_zeros = np.where(np.any(cairo_array, axis=2))
            img[non_zeros[0], non_zeros[1], :] = cairo_array[non_zeros[0], non_zeros[1], :3]
            self.output[0] = img
            return super(BinarySurface, self).finish()

    # output list is a hack to get the result out without writing into datastream
    output = [None]
    BinarySurface.convert(bytestring=svg_string, write_to=output)

    assert output[0] is not None
    return output[0]


def render_svg(svg_string):
    '''
    Render svg filename to
    :param name:
    :return:
    '''
    converters = (
        render_svg_pyside,
        render_svg_cairosvg,
    )
    exceptions = []
    for c in converters:
        try:
            start_time = time.time()
            result = c(svg_string)
            print('Took %s to convert' % (time.time() - start_time,))
            return result
        except:
            import traceback
            exceptions.append(traceback.format_exc())

    raise Exception("Failed to convert svg to numpy. Here is the list of exceptions from converters:\n%s" % ('/n'.join(exceptions),))


def create_svg_drawing():
    import svgwrite

    class RawSVGDrawing(svgwrite.container.SVG, svgwrite.elementfactory.ElementFactory):
        pass

    return RawSVGDrawing()


def svg_rectangle((x, y), (width, height), (image_width, image_height), **extra):
    '''
    Generate an svg rectangle with x, y in image coordinates in bottom-left coordinate system (!)
    :param x, y: left-bottom corner
    :param width, height: dimentions
    :return: string to add to svg xml
    '''
    drawing = create_svg_drawing()
    return drawing.rect(
        (x, image_height-y),
        size=(width, -height), **extra).tostring()
