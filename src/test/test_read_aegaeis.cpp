#include "../file-io/fmi_file_io_impl.h"
#include "test.h"
#include "../file-io/triangulation_file_io_impl.h"

void
test_read_aegaeis_fmi(const std::string &path) {
  std::string filename = path + "/coastline/coastline.graph";
  //std::string filename = path + "/aegaeis/pruned.graph";

  auto c = [&](const std_graph_t &g) { return check_graph (g); };
  test_read<triangulation_file_io, node_t, edge_t>(filename, c);
}

void
test_read_toy_fmi(const std::string &path) {
  std::string filename = path + "/aegaeis/toy.m_adj_list";

  auto c = [&](const std_graph_t &g) { return check_graph (g); };
  test_read<triangulation_file_io, node_t, edge_t>(filename, c);
}


int
main(int argc, char const *argv[]) {
  test_read_toy_fmi(argv[1]);
  test_read_aegaeis_fmi(argv[1]);
}
