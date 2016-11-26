cmake_minimum_required (VERSION 2.6)
project(SparseRRT)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")

add_definitions(-O3 )

IF(APPLE)

   # message( FATAL_ERROR "APPLE" )
   # Mac OS X bundle specific settings
   # set(MACOSX_BUNDLE true)
   # set(MACOSX_BUNDLE_BUNDLE_NAME ${MY_APP_NAME})

   # set(CMAKE_MACOSX_RPATH 1)

ENDIF(APPLE)


find_package(PythonLibs REQUIRED)
include_directories(${PYTHON_INCLUDE_DIRS})

find_package(Boost COMPONENTS program_options python REQUIRED)

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

#include_directories(${Boost_INCLUDE_DIR})
include_directories("${PROJECT_SOURCE_DIR}/include")
include_directories(${Numpy_INCLUDE_DIRS})

message( FATAL_ERROR "AAAA ${Numpy_INCLUDE_DIRS} BBB" )

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
    #${PROJECT_SOURCE_DIR}/src/utilities/parameter_reader.cpp
    ${PROJECT_SOURCE_DIR}/src/utilities/random.cpp
    ${PROJECT_SOURCE_DIR}/src/utilities/timer.cpp
)
#target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES})

#file(GLOB files ${PROJECT_SOURCE_DIR}/tests/*.cpp)

#foreach(f ${files})
#GET_FILENAME_COMPONENT(fname ${f} NAME_WE )
#add_executable(${fname} ${f} )
#target_link_libraries(${fname} ${PROJECT_NAME})
#endforeach()


add_library(sst_module SHARED
    ${PROJECT_SOURCE_DIR}/src/motion_planners/sst_wrapper.cpp)
target_link_libraries(sst_module ${Boost_LIBRARIES} ${PYTHON_LIBRARIES})

# Don't prepend wrapper library name with lib and add to Python libs.
set_target_properties(sst_module PROPERTIES
        PREFIX "_"
        SUFFIX ".so"
        #LIBRARY_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_PYTHON_DESTINATION}
        )
