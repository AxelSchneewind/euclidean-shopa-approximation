#include "../file-io/fmi_file_io_impl.h"
#include "test.h"

static std::vector<distance_t> expected_distances;
static std::vector<Query> queries;

void
test_route_stgtregbz_fmi (std::string path)
{
  std::string const filename = path + "/stgtregbz.fmi";
  std::ifstream input (filename);

  std::cout << "reading from " << filename << std::flush;

  fmi_file_io reader;
  std::shared_ptr<const std_graph_t> const graph_ptr(new std_graph_t (reader.read<node_t, edge_t> (input)));
  std_routing_t router (graph_ptr);

  std::cout << " done" << std::endl;

  test_routing (*graph_ptr, router, queries, expected_distances);

  std::cout << "routing in " << filename << " successful" << std::endl;
  int t = graph_ptr.use_count();
};

void
test_route_stgtregbz_ch (std::string path)
{
  std::string const filename = path + "/stgtregbz.sch";
  std::ifstream input (filename);

  std::cout << "reading from " << filename << std::flush;

  fmi_file_io reader;
  std::shared_ptr<const ch_graph_t> const graph_ptr(new ch_graph_t (reader.read<ch_node_t, ch_edge_t> (input)));
  ch_routing_t router (graph_ptr);

  std::cout << " done" << std::endl;

  test_routing (*graph_ptr, router, queries, expected_distances);

  std::cout << "routing in " << filename << " successful" << std::endl;
};

int
main (int argc, char const *argv[])
{
  std::ifstream query_stream;
  query_stream.open (std::string (argv[2]) + "/stgtregbz.que");

  std::ifstream distances;
  distances.open (std::string (argv[2]) + "/stgtregbz.sol");

  queries = get_queries (query_stream);
  expected_distances = get_distances (distances);
  test_route_stgtregbz_fmi (argv[1]);
  test_route_stgtregbz_ch (argv[1]);
}
