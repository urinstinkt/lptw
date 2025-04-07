# tw-lines
***Line planning on all paths* accelerated using *tree decomposition*.**
* Main paper: https://doi.org/10.4230/OASIcs.ATMOS.2023.4
* Experiment paper: *to be published*

## Building
Requirements:
* CMake
* C++20 compliant compiler
* Java
* Gurobi
* GraphViz (optional)

Run CMake to build the project, for example: <br>
`cmake -DCMAKE_BUILD_TYPE=Release -S . -B cmake-build-release` <br>
`cmake --build cmake-build-release`

## Usage
The build target `LP_TW2ILP` is our algorithm *tw-lines*. 
Running it without any parameters gives the following quick info:
```
usage: LP_TW2ILP <input_folder> <optional parameters>
optional parameters:
  -t<value>: time limit for ILP solving, in seconds
  -mg<value>: relative MIP optimality gap (Gurobi MIPGap)
  -td-default: disable specialized tree decomposition algorithms
  -no-viz: disable visualization output
outputs:
  solution:           <input_folder>/line-planning/Line-Concept.lin
  tree decomposition: <input_folder>/line-planning/out.td
  visualizations:
    line concept:     <input_folder>/graphics/line-plan.png
    tree decomp.:     <input_folder>/graphics/td.png
```

### Input
`LP_TW2ILP` takes input in the form of a [LinTim](https://lintim.net/) dataset.
A dataset must contain graph data and edge weights in the files `basis/Edge.giv` and `basis/Load.giv`.
The following config values must be present, defined in `basis/Config.cnf`:

* `lpool_costs_fixed`
* `lpool_costs_length`
* `lpool_costs_edges`

For the visualization it also needs the config `ptn_draw_conversion_factor` and
the file `basis/Stop.giv`.

We included two artificial sample datasets in the folder `example-data`.
See the documentation of LinTim for more details.

### Output
`LP_TW2ILP` solves _(LPAL)_ and outputs an optimal line plan/concept in the LinTim-native format.
It additionally outputs the tree decomposition of the input graph and visualizations
of the line plan and the tree decomposition.
LinTim also comes with a nice line plan visualization tool, which you may prefer over ours 😉

## Experiments
TODO