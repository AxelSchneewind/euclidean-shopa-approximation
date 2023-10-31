#include "../file-io/fmi_file_io_impl.h"
#include "../file-io/triangulation_file_io_impl.h"
#include "test.h"

void
test_route_aegaeis (std::string __path)
{
  std::string filename = __path + "/aegaeis/pruned.graph";
  std::ifstream input;
  input.open (filename);
  triangulation_file_io reader;
  std::shared_ptr<const std_graph_t> graph_ptr (new std_graph_t (reader.read<std_graph_t> (input)));

  // prepare queries
  std::vector<node_id_t> nodes ({ 1238, 10239, 1241, 154345, 11488, 89585, 500000, 254000, 658000 });
  std::vector<Query> queries;
  for (auto src : nodes)
    for (auto dest : nodes)
      queries.push_back (Query{ src, dest });

  std_routing_t router (graph_ptr);
  a_star_routing_t a_star_router (graph_ptr);
  compare_routing(*graph_ptr, router, a_star_router, queries);
};

void
test_route_toy (std::string __path)
{
  std::string filename = __path + "/aegaeis/toy.graph";
  std::ifstream input;
  input.open (filename);
  triangulation_file_io reader;
  std::shared_ptr<const std_graph_t> graph_ptr (new std_graph_t (reader.read<std_graph_t> (input)));
  std_routing_t router (graph_ptr);

  std::vector<Query> queries;
  for (int src = 0; src < graph_ptr->node_count (); ++src)
    for (int dest = 0; dest < graph_ptr->node_count (); ++dest)
      queries.push_back (Query{ src, dest });

  test_routing (*graph_ptr, router, queries);

  std::cout << "routing in " << filename << " successful" << std::endl;
  return;
};

int
main (int argc, char const *argv[])
{
  test_route_toy (argv[1]);
  test_route_aegaeis (argv[1]);
}
