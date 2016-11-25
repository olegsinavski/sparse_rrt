# Sparse-RRT Package


## COMPILING
To build the package, follow these steps in the  package directory:

```
mkdir build
cd build
cmake ..
make
```

This will create a bin directory in the package directory where an executable
run is created. 

## EXECUTING
To run with default parameters from input/default.cfg, just run 
this executable.

```
cd ../bin
./run
```

## INPUT

This executable takes several options, but it is possible to 
put all of these options into a configuration file and pass that as an argument:

```
./run --config=RELATIVE_PATH_TO_FILE
```

For an example of the parameters you can set, you may check the default
configuration file:

```
more ../input/default.cfg
```

This configuration file performs planning for a simple point system using SST.
The planner runs for 15 seconds and then reports the resulting solution. Other 
configuration files can be found in the input directory.

All of the parameters can be found by running

```
./run --help
```

The output of this command is shown here for reference: 

```
Options:
  --help                                Print available options.
  --config arg (=../input/default.cfg)  The name of a file to read for options 
                                        (default is ../input/default.cfg). 
                                        Command-line options override the ones 
                                        in the config file. A config file may 
                                        contain lines with syntax
                                        'long_option_name = value'
                                        and comment lines that begin with '#'.
  --integration_step arg                Integration step for propagations.
  --stopping_type arg                   Condition for terminating planner 
                                        (iterations or time).
  --stopping_check arg                  Amount of time or iterations to 
                                        execute.
  --stats_type arg                      Condition for printing statistics of a 
                                        planner (iterations or time).
  --stats_check arg                     Frequency of statistics gathering.
  --intermediate_visualization arg (=0) Flag denoting generating images during 
                                        statistics gathering.
  --min_time_steps arg                  Minimum number of simulation steps per 
                                        local planner propagation.
  --max_time_steps arg                  Maximum number of simulation steps per 
                                        local planner propagation.
  --random_seed arg                     Random seed for the planner.
  --sst_delta_near arg                  The radius for BestNear in SST.
  --sst_delta_drain arg                 The radius for witness nodes in SST.
  --planner arg                         A string for the planner to run.
  --system arg                          A string for the system to plan for.
  --start_state arg                     The given start state. Input is in the 
                                        format of "0 0"
  --goal_state arg                      The given goal state. Input is in the 
                                        format of "0 0"
  --goal_radius arg                     The radius for the goal region.
  --tree_line_width arg                 Line thickness for tree visualization.
  --solution_line_width arg             Line thickness for solution path.
  --image_width arg                     Width of output images.
  --image_height arg                    Height of output images.
  --node_diameter arg                   Diameter of visualized nodes in output 
                                        images.
  --solution_node_diameter arg          Diameter of nodes along solution path 
                                        in output images.
```

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

## EDITING CODE

Header files can be found in the include directory. The corresponding source
files are found in the src directory. Adding new files into these directories 
should be included in any subsequent builds of the code, provided that cmake
is invoked again as detailed above in the COMPILING section. 

New executables can be placed in the tests directory, where the run executable 
source code can be found.