cmake_minimum_required(VERSION 2.8)
project(SparseRRT)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
set(CMAKE_CXX_FLAGS "-std=c++1y ${CMAKE_CXX_FLAGS}")
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

set(INCLUDE_DIRECTORIES include)

add_definitions(-O3 )

IF(APPLE)
   set(CMAKE_MACOSX_RPATH 1)
ENDIF(APPLE)

# Instead of find_package(PythonLibs REQUIRED), we manually search for python to make sure we link to the active version
# (as opposed to system python if brew python is installed)
execute_process(
    COMMAND "python2.7" -c "import sys;print(sys.prefix)"
    OUTPUT_VARIABLE PYTHON_PREFIX
    OUTPUT_STRIP_TRAILING_WHITESPACE
)
if(APPLE)
    set(PYTHON_LIBRARIES "${PYTHON_PREFIX}/lib/libpython2.7.dylib")
else(APPLE)
    # it seems like libpython2.7.so is not necessary on ubuntu
    # set(PYTHON_LIBRARIES "${PYTHON_PREFIX}/lib/libpython2.7.so")
endif(APPLE)

execute_process(
    COMMAND "python2.7" -c "from distutils import sysconfig; print(sysconfig.get_python_inc())"
    OUTPUT_VARIABLE PYTHON_INCLUDE_DIRS
    OUTPUT_STRIP_TRAILING_WHITESPACE
)


include_directories(
        # ${Numpy_INCLUDE_DIRS}  # this should come before PYTHON includes because it also contains system numpy
        ${PYTHON_INCLUDE_DIRS}
        "${PROJECT_SOURCE_DIR}/include"
        "${PROJECT_SOURCE_DIR}/deps/pybind11/include"
        )


set(SOURCE_FILES
        src/motion_planners/rrt.cpp
        src/motion_planners/sst.cpp
        src/nearest_neighbors/graph_nearest_neighbors.cpp
        src/systems/car.cpp
        src/systems/cart_pole.cpp
        src/systems/pendulum.cpp
        src/systems/point.cpp
        src/systems/rally_car.cpp
        src/systems/two_link_acrobot.cpp
        src/utilities/random.cpp
        src/utilities/timer.cpp
        src/image_creation/svg_image.cpp
        src/image_creation/planner_visualization.cpp
        src/systems/distance_functions.cpp)

add_library(${PROJECT_NAME} STATIC
    ${SOURCE_FILES})

add_library(sst_module SHARED
    ${PROJECT_SOURCE_DIR}/src/python_wrapper.cpp)
target_link_libraries(sst_module ${PYTHON_LIBRARIES} ${PROJECT_NAME})

# Don't prepend wrapper library name with lib and add to Python libs.
set_target_properties(sst_module PROPERTIES
        PREFIX "_"
        SUFFIX ".so"
        LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/sparse_rrt
        )
