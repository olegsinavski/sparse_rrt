
/*******************************************************************************
 *  The "New BSD License" : http://www.opensource.org/licenses/bsd-license.php  *
 ********************************************************************************
 
 Original work Copyright (c) 2010, Mark Turney
 Modified work Copyright (c) 2010, Oleg Y. Sinyavskiy
 All rights reserved.
 
 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:
 * Redistributions of source code must retain the above copyright
 notice, this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright
 notice, this list of conditions and the following disclaimer in the
 documentation and/or other materials provided with the distribution.
 * Neither the name of the <organization> nor the
 names of its contributors may be used to endorse or promote products
 derived from this software without specific prior written permission.
 
 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
 DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 
 ******************************************************************************/

#ifndef SIMPLE_SVG_HPP
#define SIMPLE_SVG_HPP

#include <vector>
#include <string>
#include <sstream>
#include <fstream>
#include <stdexcept>

#include <iostream>
#include <tuple>

namespace svg
{
    // Utility XML/String Functions.
    template <typename T>
    std::string attribute(std::string const & attribute_name,
                          T const & value, std::string const & unit = "")
    {
        std::stringstream ss;
        ss << attribute_name << "=\"" << value << unit << "\" ";
        return ss.str();
    }

    template <typename T>
    class attribute_streamer
    {
    public:
        attribute_streamer(
            std::string const & attribute_name,
            T const & value, std::string const & unit = "")
            : _attribute_name(attribute_name)
            , _value(value)
            , _unit(unit)
        {}

        template<typename aT>
        friend std::ostream & operator << (std::ostream &out, const attribute_streamer<aT> &c);

    private:
        std::string const & _attribute_name;
        T const & _value;
        std::string const & _unit;
    };

    template<typename T>
    std::ostream & operator << (std::ostream &out, const attribute_streamer<T> &c)
    {
        out << c._attribute_name << "=\"" << c._value << c._unit << "\" ";
        return out;
    }

    template <typename T>
    attribute_streamer<T> stream_attribute(std::string const & attribute_name,
                                           T const & value, std::string const & unit = "")
    {
        return std::move(attribute_streamer<T>(attribute_name, value, unit));
    }

    std::string elemStart(std::string const & element_name);
    std::string elemEnd(std::string const & element_name);
    std::string emptyElemEnd();
    
    // Quick optional return type.  This allows functions to return an invalid
    //  value if no good return is possible.  The user checks for validity
    //  before using the returned value.
    template <typename T>
    class optional
    {
    public:
        optional<T>(T const & type)
        : valid(true), type(type) { }
        optional<T>() : valid(false), type(T()) { }
        T * operator->()
        {
            // If we try to access an invalid value, an exception is thrown.
            if (!valid)
                throw std::exception();
            
            return &type;
        }
        // Test for validity.
        bool operator!() const { return !valid; }
    private:
        bool valid;
        T type;
    };
    
    struct Dimensions
    {
        Dimensions(double width, double height) : width(width), height(height) { }
        Dimensions(double combined = 0) : width(combined), height(combined) { }
        double width;
        double height;
    };
    
    struct Point
    {
        Point(double x = 0, double y = 0) : x(x), y(y) { }
        double x;
        double y;
    };
    optional<Point> getMinPoint(std::vector<Point> const & points);
    optional<Point> getMaxPoint(std::vector<Point> const & points);
    
    // Defines the dimensions, scale, origin, and origin offset of the document.
    struct Layout
    {
        enum Origin { TopLeft, BottomLeft, TopRight, BottomRight };
        
        Layout(Dimensions const & dimensions = Dimensions(400, 300), Origin origin = BottomLeft,
               double scale = 1, Point const & origin_offset = Point(0, 0))
        : dimensions(dimensions), scale(scale), origin(origin), origin_offset(origin_offset) { }
        Dimensions dimensions;
        double scale;
        Origin origin;
        Point origin_offset;
    };
    
    // Convert coordinates in user space to SVG native space.
    double translateX(double x, Layout const & layout);
    
    double translateY(double y, Layout const & layout);
    double translateScale(double dimension, Layout const & layout);
    
    class Serializeable
    {
    public:
        Serializeable() { }
        virtual ~Serializeable() { };
        virtual void toStream(std::ostream& output, Layout const & layout) const = 0;
    };

    class stream
    {
    public:
        stream(
            Serializeable const & serializeable,
            Layout const & layout)
            : _serializeable(serializeable)
            , _layout(layout)
        {}

        friend std::ostream & operator << (std::ostream &out, const stream &streamer);
    private:
        Serializeable const & _serializeable;
        Layout const & _layout;
    };

    inline std::ostream & operator << (std::ostream &out, const stream &streamer)
    {
        streamer._serializeable.toStream(out, streamer._layout);
        return out;
    }

    class Color : public Serializeable
    {
    public:
        enum Defaults { Transparent = -1, Aqua, Black, Blue, Brown, Cyan, Fuchsia,
            Green, Lime, Magenta, Orange, Purple, Red, Silver, White, Yellow };
        
        Color(int r, int g, int b) : transparent(false), red(r), green(g), blue(b) { }
        Color(Defaults color)
        : transparent(false), red(0), green(0), blue(0)
        {
            switch (color)
            {
                case Aqua: assign(0, 255, 255); break;
                case Black: assign(0, 0, 0); break;
                case Blue: assign(0, 0, 255); break;
                case Brown: assign(165, 42, 42); break;
                case Cyan: assign(0, 255, 255); break;
                case Fuchsia: assign(255, 0, 255); break;
                case Green: assign(0, 128, 0); break;
                case Lime: assign(0, 255, 0); break;
                case Magenta: assign(255, 0, 255); break;
                case Orange: assign(255, 165, 0); break;
                case Purple: assign(128, 0, 128); break;
                case Red: assign(255, 0, 0); break;
                case Silver: assign(192, 192, 192); break;
                case White: assign(255, 255, 255); break;
                case Yellow: assign(255, 255, 0); break;
                default: transparent = true; break;
            }
        }
        virtual ~Color() { }

        void toStream(std::ostream& ss, Layout const & layout) const {
            if (transparent)
                ss << "transparent";
            else
                ss << "rgb(" << red << "," << green << "," << blue << ")";
        }
    private:
        bool transparent;
        int red;
        int green;
        int blue;
        
        void assign(int r, int g, int b)
        {
            red = r;
            green = g;
            blue = b;
        }
    };
    
    class Fill : public Serializeable
    {
    public:
        Fill(Color::Defaults color) : color(color) { }
        Fill(Color color = Color::Transparent)
        : color(color) { }

        void toStream(std::ostream& ss, Layout const & layout) const {
            ss << stream_attribute("fill", stream(color, layout));
        }

    private:
        Color color;
    };
    
    class Stroke : public Serializeable
    {
    public:
        Stroke(double width = -1, Color color = Color::Transparent)
        : width(width), color(color) { }

        void toStream(std::ostream& ss, Layout const & layout) const {
            // If stroke width is invalid.
            if (width < 0)
                return;

            ss << attribute("stroke-width", translateScale(width, layout)) << attribute("stroke", stream(color, layout));
        }

    private:
        double width;
        Color color;
    };
    
    class Font : public Serializeable
    {
    public:
        Font(double size = 12, std::string const & family = "Verdana") : size(size), family(family) { }

        void toStream(std::ostream& ss, Layout const & layout) const {
            ss << attribute("font-size", translateScale(size, layout)) << attribute("font-family", family);
        }

    private:
        double size;
        std::string family;
    };
    
    class Shape : public Serializeable
    {
    public:
        Shape(Fill const & fill = Fill(), Stroke const & stroke = Stroke())
        : fill(fill), stroke(stroke) { }
        virtual ~Shape() { }
        virtual void offset(Point const & offset) = 0;
    protected:
        Fill fill;
        Stroke stroke;
    };

    template <typename T>
    void vectorToStream(std::ostream& ss, std::vector<T> collection, Layout const & layout)
    {

    }
    
    class Circle : public Shape
    {
    public:
        Circle(Point const & center, double diameter, Fill const & fill,
               Stroke const & stroke = Stroke())
        : Shape(fill, stroke), center(center), radius(diameter / 2) { }

        void toStream(std::ostream& ss, Layout const & layout) const {
            ss << elemStart("circle") << attribute("cx", translateX(center.x, layout))
               << attribute("cy", translateY(center.y, layout))
               << attribute("r", translateScale(radius, layout)) << stream(fill, layout)
               << stream(stroke, layout) << emptyElemEnd();
        }

        void offset(Point const & offset)
        {
            center.x += offset.x;
            center.y += offset.y;
        }
    private:
        Point center;
        double radius;
    };
    
    class Elipse : public Shape
    {
    public:
        Elipse(Point const & center, double width, double height,
               Fill const & fill = Fill(), Stroke const & stroke = Stroke())
        : Shape(fill, stroke), center(center), radius_width(width / 2),
        radius_height(height / 2) { }

        void toStream(std::ostream& ss, Layout const & layout) const {
            ss << elemStart("ellipse") << attribute("cx", translateX(center.x, layout))
               << attribute("cy", translateY(center.y, layout))
               << attribute("rx", translateScale(radius_width, layout))
               << attribute("ry", translateScale(radius_height, layout))
               << stream(fill, layout) << stream(stroke, layout) << emptyElemEnd();
        }
        void offset(Point const & offset)
        {
            center.x += offset.x;
            center.y += offset.y;
        }
    private:
        Point center;
        double radius_width;
        double radius_height;
    };
    
    class Rectangle : public Shape
    {
    public:
        Rectangle(Point const & edge, double width, double height,
                  Fill const & fill = Fill(), Stroke const & stroke = Stroke())
        : Shape(fill, stroke), edge(edge), width(width),
        height(height) { }

        void toStream(std::ostream& ss, Layout const & layout) const {
            ss << elemStart("rect") << attribute("x", translateX(edge.x, layout))
               << attribute("y", translateY(edge.y, layout))
               << attribute("width", translateScale(width, layout))
               << attribute("height", translateScale(height, layout))
               << stream(fill, layout) << stream(stroke, layout) << emptyElemEnd();
        }

        void offset(Point const & offset)
        {
            edge.x += offset.x;
            edge.y += offset.y;
        }
    private:
        Point edge;
        double width;
        double height;
    };
    
    class Line : public Shape
    {
    public:
        Line(Point const & start_point, Point const & end_point,
             Stroke const & stroke = Stroke())
        : Shape(Fill(), stroke), start_point(start_point),
        end_point(end_point) { }

        void toStream(std::ostream& ss, Layout const & layout) const {
            ss << elemStart("line") << attribute("x1", translateX(start_point.x, layout))
               << attribute("y1", translateY(start_point.y, layout))
               << attribute("x2", translateX(end_point.x, layout))
               << attribute("y2", translateY(end_point.y, layout))
               << stream(stroke, layout) << emptyElemEnd();
        }

        void offset(Point const & offset)
        {
            start_point.x += offset.x;
            start_point.y += offset.y;
            
            end_point.x += offset.x;
            end_point.y += offset.y;
        }
    private:
        Point start_point;
        Point end_point;
    };
    
    class Polygon : public Shape
    {
    public:
        Polygon(Fill const & fill = Fill(), Stroke const & stroke = Stroke())
        : Shape(fill, stroke) { }
        Polygon(Stroke const & stroke = Stroke()) : Shape(Color::Transparent, stroke) { }
        Polygon & operator<<(Point const & point)
        {
            points.push_back(point);
            return *this;
        }

        void toStream(std::ostream& ss, Layout const & layout) const {
            ss << elemStart("polygon");

            ss << "points=\"";
            for (unsigned i = 0; i < points.size(); ++i)
                ss << translateX(points[i].x, layout) << "," << translateY(points[i].y, layout) << " ";
            ss << "\" ";

            ss << stream(fill, layout) << stream(stroke, layout) << emptyElemEnd();
        }

        void offset(Point const & offset)
        {
            for (unsigned i = 0; i < points.size(); ++i) {
                points[i].x += offset.x;
                points[i].y += offset.y;
            }
        }
    private:
        std::vector<Point> points;
    };
    
    class Polyline : public Shape
    {
    public:
        Polyline(Fill const & fill = Fill(), Stroke const & stroke = Stroke())
        : Shape(fill, stroke) { }
        Polyline(Stroke const & stroke = Stroke()) : Shape(Color::Transparent, stroke) { }
        Polyline(std::vector<Point> const & points,
                 Fill const & fill = Fill(), Stroke const & stroke = Stroke())
        : Shape(fill, stroke), points(points) { }
        Polyline & operator<<(Point const & point)
        {
            points.push_back(point);
            return *this;
        }

        void toStream(std::ostream& ss, Layout const & layout) const {
            ss << elemStart("polyline");

            ss << "points=\"";
            for (unsigned i = 0; i < points.size(); ++i)
                ss << translateX(points[i].x, layout) << "," << translateY(points[i].y, layout) << " ";
            ss << "\" ";

            ss << stream(fill, layout) << stream(stroke, layout) << emptyElemEnd();
        }

        void offset(Point const & offset)
        {
            for (unsigned i = 0; i < points.size(); ++i) {
                points[i].x += offset.x;
                points[i].y += offset.y;
            }
        }
        std::vector<Point> points;
    };
    
    class Text : public Shape
    {
    public:
        Text(Point const & origin, std::string const & content, Fill const & fill = Fill(),
             Font const & font = Font(), Stroke const & stroke = Stroke())
        : Shape(fill, stroke), origin(origin), content(content), font(font) { }

        void toStream(std::ostream& ss, Layout const & layout) const {
            ss << elemStart("text") << attribute("x", translateX(origin.x, layout))
               << attribute("y", translateY(origin.y, layout))
               << stream(fill, layout) << stream(stroke, layout) << stream(font, layout)
               << ">" << content << elemEnd("text");
        }

        void offset(Point const & offset)
        {
            origin.x += offset.x;
            origin.y += offset.y;
        }
    private:
        Point origin;
        std::string content;
        Font font;
    };

    // Sample charting class.
    class LineChart : public Shape
    {
    public:
        LineChart(Dimensions margin = Dimensions(), double scale = 1,
                  Stroke const & axis_stroke = Stroke(.5, Color::Purple))
        : axis_stroke(axis_stroke), margin(margin), scale(scale) { }
        LineChart & operator<<(Polyline const & polyline)
        {
            if (polyline.points.empty())
                return *this;

            polylines.push_back(polyline);
            return *this;
        }

        void toStream(std::ostream& ss, Layout const & layout) const {
            if (polylines.empty())
                return;

            for (unsigned i = 0; i < polylines.size(); ++i)
                polylineToStream(ss, polylines[i], layout);

            axisString(ss, layout);
        }

        void offset(Point const & offset)
        {
            for (unsigned i = 0; i < polylines.size(); ++i)
                polylines[i].offset(offset);
        }
    private:
        Stroke axis_stroke;
        Dimensions margin;
        double scale;
        std::vector<Polyline> polylines;

        optional<Dimensions> getDimensions() const
        {
            if (polylines.empty())
                return optional<Dimensions>();

            optional<Point> min = getMinPoint(polylines[0].points);
            optional<Point> max = getMaxPoint(polylines[0].points);
            for (unsigned i = 0; i < polylines.size(); ++i) {
                if (getMinPoint(polylines[i].points)->x < min->x)
                    min->x = getMinPoint(polylines[i].points)->x;
                if (getMinPoint(polylines[i].points)->y < min->y)
                    min->y = getMinPoint(polylines[i].points)->y;
                if (getMaxPoint(polylines[i].points)->x > max->x)
                    max->x = getMaxPoint(polylines[i].points)->x;
                if (getMaxPoint(polylines[i].points)->y > max->y)
                    max->y = getMaxPoint(polylines[i].points)->y;
            }

            return optional<Dimensions>(Dimensions(max->x - min->x, max->y - min->y));
        }
        void axisString(std::ostream& ss, Layout const & layout) const
        {
            optional<Dimensions> dimensions = getDimensions();
            if (!dimensions)
                return;

            // Make the axis 10% wider and higher than the data points.
            double width = dimensions->width * 1.1;
            double height = dimensions->height * 1.1;

            // Draw the axis.
            Polyline axis(Color::Transparent, axis_stroke);
            axis << Point(margin.width, margin.height + height) << Point(margin.width, margin.height)
                 << Point(margin.width + width, margin.height);

            ss << stream(axis, layout);
        }
        void polylineToStream(std::ostream& ss, Polyline const & polyline, Layout const & layout) const
        {
            Polyline shifted_polyline = polyline;
            shifted_polyline.offset(Point(margin.width, margin.height));
            ss << stream(shifted_polyline, layout);

            for (unsigned i = 0; i < shifted_polyline.points.size(); ++i) {
                auto circle = Circle(shifted_polyline.points[i], getDimensions()->height / 30.0, Color::Black);
                ss << stream(circle, layout);
            }
        }
    };

    class Header : public Serializeable
    {
    public:
        void toStream(std::ostream& ss, Layout const & layout) const {
            ss << "<?xml " << attribute("version", "1.0") << attribute("standalone", "no")
            << "?>\n<!DOCTYPE svg PUBLIC \"-//W3C//DTD SVG 1.1//EN\" "
            << "\"http://www.w3.org/Graphics/SVG/1.1/DTD/svg11.dtd\">\n<svg "
            << attribute("width", layout.dimensions.width, "px")
            << attribute("height", layout.dimensions.height, "px")
            << attribute("xmlns", "http://www.w3.org/2000/svg")
            << attribute("version", "1.1") << ">\n";
        }
    };

    class Footer : public Serializeable
    {
    public:
        void toStream(std::ostream& ss, Layout const & layout) const {
            ss << elemEnd("svg");
        }
    };

    class DocumentBody
    {
    public:
        DocumentBody(Layout layout = Layout())
        : layout(layout) { }

        DocumentBody & operator<<(Shape const & shape)
        {
            shape.toStream(body_nodes_stream, layout);
            return *this;
        }

        std::string  toString() const
        {
            return std::move(this->body_nodes_stream.str());
        }
    private:
        Layout layout;

        std::stringstream body_nodes_stream;
    };

    class Document
    {
    public:
        Document(Layout layout = Layout())
        : layout(layout) { }
        
        Document & operator<<(Shape const & shape)
        {
            shape.toStream(body_nodes_stream, layout);
            return *this;
        }
        std::tuple<std::string, std::string, std::string>  toString() const
        {
            std::stringstream ss_header;
            Header().toStream(ss_header, layout);

            std::stringstream ss_footer;
            Footer().toStream(ss_footer, layout);
            return std::make_tuple(ss_header.str(), this->body_nodes_stream.str(), ss_footer.str());
        }
    private:
        Layout layout;

        std::stringstream body_nodes_stream;
    };
}

#endif
