# Triangulation

This directory contains the code most relevant for the approximation scheme.

`steiner_graph.h` defines the implicit/semi-explicit graph representations, based
on an adjacency list, a polyhedron (`polyhedron.h`) the information on Steiner point placement (`subdivision_info.h`).
`steiner_neighbors.h` defines the different pruning variants.

`steiner_labels.h` and `frontier_labels.h` define the data structures for storing
tree information on the discretization.

`subdivision_table.h` provides an alternative method of storing Steiner point placement, 
based on tables of relative positions. However, this is not guaranteed to still be functional.