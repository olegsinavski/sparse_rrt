cmake_minimum_required (VERSION 2.6)
project(SparseRRT)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")

add_definitions(-O3 )

IF(APPLE)
   set(CMAKE_MACOSX_RPATH 1)
ENDIF(APPLE)

find_package(Boost COMPONENTS program_options python REQUIRED)

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

# Find actual numpy in use (as opposed to system numpy)
execute_process(
COMMAND "python2.7" -c
        "from __future__ import print_function\ntry: import numpy; print(numpy.__version__, end='')\nexcept:pass\n"
OUTPUT_VARIABLE Numpy_VERSION)
set( REQUIRED_NUMPY_VERSION "1.11.2" )
if( NOT (${Numpy_VERSION} STREQUAL ${REQUIRED_NUMPY_VERSION}) )
  message( FATAL_ERROR "Bad numpy version ${Numpy_VERSION}. Please run 'sudo pip install numpy==${REQUIRED_NUMPY_VERSION}'" )
endif()
execute_process(
COMMAND "python2.7" -c
        "from __future__ import print_function\ntry: import numpy; print(numpy.get_include(), end='')\nexcept:pass\n"
        OUTPUT_VARIABLE Numpy_INCLUDE_DIRS)

include_directories(
        ${Boost_INCLUDE_DIRS}
        ${Numpy_INCLUDE_DIRS}  # this should come before PYTHON includes because it also contains system numpy
        ${PYTHON_INCLUDE_DIRS}
        "${PROJECT_SOURCE_DIR}/include"
        )

add_library(${PROJECT_NAME} STATIC
    ${PROJECT_SOURCE_DIR}/src/image_creation/svg_image.cpp
    ${PROJECT_SOURCE_DIR}/src/motion_planners/planner.cpp
    ${PROJECT_SOURCE_DIR}/src/motion_planners/rrt.cpp
    ${PROJECT_SOURCE_DIR}/src/motion_planners/sst.cpp
    ${PROJECT_SOURCE_DIR}/src/nearest_neighbors/graph_nearest_neighbors.cpp
    ${PROJECT_SOURCE_DIR}/src/systems/car.cpp
    ${PROJECT_SOURCE_DIR}/src/systems/cart_pole.cpp
    ${PROJECT_SOURCE_DIR}/src/systems/pendulum.cpp
    ${PROJECT_SOURCE_DIR}/src/systems/point.cpp
    ${PROJECT_SOURCE_DIR}/src/systems/rally_car.cpp
    ${PROJECT_SOURCE_DIR}/src/systems/two_link_acrobot.cpp
    ${PROJECT_SOURCE_DIR}/src/utilities/condition_check.cpp
    ${PROJECT_SOURCE_DIR}/src/utilities/parameter_reader.cpp
    ${PROJECT_SOURCE_DIR}/src/utilities/random.cpp
    ${PROJECT_SOURCE_DIR}/src/utilities/timer.cpp
)
target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES})

file(GLOB files ${PROJECT_SOURCE_DIR}/tests/*.cpp)

foreach(f ${files})
GET_FILENAME_COMPONENT(fname ${f} NAME_WE )
add_executable(${fname} ${f} )
target_link_libraries(${fname} ${PROJECT_NAME} ${Boost_LIBRARIES} ${PYTHON_LIBRARIES})
endforeach()

add_library(sst_module SHARED
    ${PROJECT_SOURCE_DIR}/src/motion_planners/sst_wrapper.cpp)
target_link_libraries(sst_module ${Boost_LIBRARIES} ${PYTHON_LIBRARIES} ${PROJECT_NAME})

# Don't prepend wrapper library name with lib and add to Python libs.
set_target_properties(sst_module PROPERTIES
        PREFIX "_"
        SUFFIX ".so"
        #LIBRARY_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_PYTHON_DESTINATION}
        )
