#include "routing.h"
#include "triangulation/granular_adjacency_list.h"
#include "triangulation/steiner_graph.h"
#include <fstream>

int main() {
  std::ifstream input("/aegaeis/toy.graph");

  triangulation_file_io t;
  auto graph = t.template read<steiner_graph>(input);

  return 0;
}