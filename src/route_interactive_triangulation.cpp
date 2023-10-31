#include <format>
#include <fstream>

#include "file-io/fmi_file_io.h"
#include "file-io/gl_file_io.h"
#include "file-io/triangulation_file_io.h"
#include "graph/graph.h"

#include "file-io/gl_file_io_impl.h"
#include "file-io/triangulation_file_io_impl.h"

#include "routing.h"

#include <chrono>

int
main (int argc, char const *argv[])
{
  // std::string directory = std::string (argv[1]) + "/aegaeis/";
  // std::string graph_filename = "pruned.graph";

  std::string graph_file;
  std::cout << "graph file: ";
  std::cin >> graph_file;

  std::string output_directory;
  std::cout << "output directory: ";
  std::cin >> output_directory;

  // read triangulation
  std::cout << "reading graph from " << graph_file << "... " << std::flush;
  std::ifstream input (graph_file);

  if (input.bad ())
    return 1;

  triangulation_file_io reader;

  std::shared_ptr<const std_graph_t> graph_ptr (new std_graph_t (reader.read<std_graph_t> (input)));
  std::cout << "done, graph has " << graph_ptr->node_count() << " nodes and " << graph_ptr->edge_count() << " edges" << std::endl;

  std_routing_t router (graph_ptr);
  a_star_routing_t a_star_router (graph_ptr);

  while (true)
  {
    // get query
    node_id_t src, dest;
    char mode;
    std::cout << "src node: " << std::flush;
    std::cin >> src;
    std::cout << "dest node: " << std::flush;
    std::cin >> dest;
    std::cout << "mode (B = Bidirectional Dijkstra, A = A*) : " << std::flush;
    std::cin >> mode;

    // check that query is valid
    if (src < 0 || dest < 0 || src >= graph_ptr->node_count () || dest >= graph_ptr->node_count ())
      break;

    // setup writer for graphs to show
    std::string route_file = std::format ("{}/route_{}_{}_{}.gl", output_directory, mode, src, dest);
    std::string tree_file = std::format ("{}/tree_{}_{}_{}.gl", output_directory, mode, src, dest);
    std::string info_file = std::format ("{}/info_{}_{}_{}.gl", output_directory, mode, src, dest);
    std::ofstream output_route (route_file);
    std::ofstream output_tree (tree_file);
    std::ofstream output_info (info_file);
    gl_file_io writer;

    //
    std_graph_t::path route;
    std_graph_t::subgraph tree_subgraph;
    std::chrono::time_point<std::chrono::system_clock, std::chrono::duration<double>> before;
    std::chrono::time_point<std::chrono::system_clock, std::chrono::duration<double>> after;

    switch (mode)
    {
    case 'B':
      router.init (src, dest);
      before = std::chrono::high_resolution_clock::now ();
      router.compute_route ();
      after = std::chrono::high_resolution_clock::now ();

      if (router.route_found ())
      {
	route = router.route ();
	tree_subgraph = router.shortest_path_tree ();
      }
      break;

    case 'A':
      a_star_router.init (src, dest);
      before = std::chrono::high_resolution_clock::now ();
      a_star_router.compute_route ();
      after = std::chrono::high_resolution_clock::now ();

      if (a_star_router.route_found ())
      {
	route = a_star_router.route ();
	tree_subgraph = a_star_router.shortest_path_tree ();
      }
      break;
    default:
      break;
    }

    // get time
    std::chrono::duration<double, std::milli> routing_time = after - before;

    // make graph from route to display
    auto route_subgraph = graph_ptr->make_subgraph (route);
    auto route_graph = graph_ptr->make_graph (route_subgraph);

    // make graph from shortest path tree
    auto tree_graph = graph_ptr->make_graph (tree_subgraph);

    // write output graphs
    if (route_graph.node_count () > 0)
    {
      writer.write (output_route, route_graph, 12, 5);
      writer.write (output_tree, tree_graph, 3, 4);
    }

    // print stats about route computation
    output_info << "path: " << route << '\n';
    output_info << "path has cost: " << graph_ptr->path_length (route) << '\n';
    output_info << "searches visited " << tree_graph.node_count () << " nodes";
    output_info << "and took " << routing_time << '\n';

    std::cout << "\tpath: " << route << '\n';
    std::cout << "\tpath has cost: " << graph_ptr->path_length (route) << '\n';
    std::cout << "\tsearches visited " << tree_graph.node_count () << " nodes";
    std::cout << "\tand took " << routing_time << '\n';

    output_route.close ();
    output_tree.close ();
    output_info.close ();
  }

  return 0;
}