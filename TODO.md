
- add subdivision tables

- Problem: routing near coasts slows down search, maybe prefer larger triangles over smaller ones with A*?

- A*:
  - IMPORTANT: compute heuristic value per triangle, not per node
  - allow push_range for A* queue to support vectorization of distance computation

- don't subdivide edges on obstacles
