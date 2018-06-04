# Python bindings for Sparse-RRT planner

This package is based on Sparse-RRT Package https://bitbucket.org/pracsys/sparse_rrt/.
The main purpose of this work is to allow running Sparse-RRT planner in python environment. 
Here are the main contributions:
 - C++ experiments from original package are executable from Python while preserving speed, SVG visualizations and statistics
 - Controllable systems for the planners can be completely written in Python, while planners are executed in C++. This is not so slow as it sounds - the average slowdown is only 2-3x for system's dynamic integration.
 - C++ code of planners, utilities and systems underwent a lot of refactoring using moder C++11 features. Python bindings are done using pybind11
 - C++ implementation of planners, utilities and systems used to contain many memory leaks that prevent running planning multiple times. This was fixed in the current implementation.


## COMPILING
To build the package, follow these steps in the  package directory:

```
mkdir build
cd build
cmake ..
make
```


## EXECUTING


## INPUT



## OUTPUT

In addition to the terminal output, visualization images in .svg format are 
placed in the bin directory. One image shows the resulting tree of the motion
planner along with the solution path (tree_*.svg). The second image (nodes_*.svg)
shows the cost at each node in the tree. Darker nodes represent lower cost (better)
while lighter nodes denote higher cost. A series of these images can be created to 
show the evolution of the planner's data structure over time using the 
intermediate_visualization parameter of the input. 

## STATISTICS

Right now, the reported statistics are the number of iterations executed, the 
number of nodes stored, and the solution length in seconds. These are printed
to the terminal at an interval set by the user with the stats_type and stats_check
parameters.

## DOCUMENTATION

Documentation for the package can be generated in the following way:

```
cd doc
doxygen doxyfile
```

Then opening html/index.html

