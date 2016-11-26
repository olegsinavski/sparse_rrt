

#include <numpy/arrayobject.h>

#include <boost/python.hpp>
#include <boost/format.hpp>
#include <iostream>
//#include "utilities/numpy_boost_python.hpp"

namespace py = boost::python;

void translate_exception(std::runtime_error const& e)
{
    // Use the Python 'C' API to set up an exception object
    PyErr_SetString(PyExc_RuntimeError, e.what());
}


class SSTWrapper
{
public:
    SSTWrapper()
    {
    }

    void run() {

    }

};


BOOST_PYTHON_MODULE(_sst_module)
{
    import_array();
//    numpy_boost_python_register_type<uint8_t, 2>();
//    numpy_boost_python_register_type<int32_t, 1>();
//    numpy_boost_python_register_type<int32_t, 2>();
//    numpy_boost_python_register_type<double, 1>();
//    numpy_boost_python_register_type<double, 2>();
//    numpy_boost_python_register_type<int, 2>();

    //py::register_exception_translator<std::runtime_error>(&translate_exception);

    py::class_<SSTWrapper, boost::noncopyable>(
        "SSTWrapper", boost::python::init<>())
            .def("run", &SSTWrapper::run)
    ;
}
