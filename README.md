# Euclidean shortest path approximation

This repository contains the code used for my thesis on the topic
`Fully-polynomial-time Approximation Schemes for the Euclidean Shortest Path Problem`.

The relevant code for the discretization is contained in the `src/triangulation` directory.
It contains the implementation of the implicit/semi-explicit graph representation 
as well as the implementation of the pruned search.

## Building
The project can be built using CMake.
It optionally requires doxygen (for documentation) and gengetopt (for modifying parsing of command line parameters) to be installed.

To build, run
```sh
mkdir cmake-build-release
cmake -B cmake-build-release -S . -DCMAKE_BUILD_TYPE=Release
```

The following executables are provided:

- `compute_shopa`: reads either a .fmi or a .graph file and computes paths between source-target-pairs
- `compute_ota`: reads either a .fmi or a .graph file and computes shortest-path-trees for given source nodes
- `graph_stats`: reads a .graph file and outputs a chosen type of information on the graph (size, inner angles, points inserted per edge)
- `find_nodes`: reads a .graph file and selects a given number of nodes with a given property
- `make_explicit`: reads a .graph file and constructs the explicit graph representation (for a given epsilon) and writes to a file
- `make_gl`: reads a .fmi or .graph file and converts it to a .gl file for rendering
- `convert`: reads a .graph file and converts it to .node and .ele files used by triangle (Shewchuck) and vice versa.
- `prune_graph`: cuts out a subgraph from a .fmi or .graph file by a given bounding box

## Computing shortest paths
Example:
```
compute_shopa --graph-file aegaeis-ref.graph -o output-dir/ -e 0.5 -q $(find_nodes --graph-file aegaeis-ref.graph -b -r 2)
```
Selects two random boundary vertices and computes a shortest path between them and writes the results to the given output directory.

## Experiments

In the experiments directory, the conducted experiments are included. They contain the respective scripts to run the experiments. For the experiments relying on specific graph instances, their paths have to be specified in experiments/utils.sh.

