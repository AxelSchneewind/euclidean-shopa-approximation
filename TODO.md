
- basics:
  - make unidirectional_adjacency_list support multiple columns
  - update interfaces, allow iteration over outgoing edge_ids

- steiner graph
  - implement quadratic function for point placements

- distance computation
  - fix issues at latitude 180
  - speed up haversine formula?

- A*:
  - IMPORTANT: compute heuristic value per triangle, not per node
- don't subdivide edges on obstacles