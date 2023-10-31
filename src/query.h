#pragma once

#include <chrono>
#include <iostream>

#include "graph/base_types.h"

#include "graph/graph.h"

struct Query
{
  node_id_t from;
  node_id_t to;
};

struct Result
{
  Query query;

  bool route_found = false;

  distance_t distance = DISTANCE_INF;
  distance_t beeline_distance = DISTANCE_INF;

  path<node_id_t> route;
  subgraph<node_id_t, edge_id_t> trees;

  size_t nodes_visited = 0;
  node_id_t mid_node = NO_NODE_ID;

  std::chrono::duration<double, std::milli> duration;
};

template <typename Graph, typename Router>
Result
perform_query (const Graph &graph, Router &router, const Query &query)
{
  Result result;
  result.query = query;
  result.beeline_distance = distance (graph.node (query.from).coordinates, graph.node (query.to).coordinates);

  router.init (query.from, query.to);

  // setup timing
  auto before
    = std::chrono::high_resolution_clock::now ();

  router.compute_route ();

  auto after
    = std::chrono::high_resolution_clock::now ();

  result.duration = after - before;
  result.route_found = router.route_found ();
  if (router.route_found ())
  {
    result.route = router.route ();
    result.distance = graph.path_length (result.route);
    result.trees = router.shortest_path_tree ();
    result.mid_node = router.mid_node ();
  }

  return result;
}

struct output
{
  bool query = true, timing = false, distance = false, mid_node = false, route = false, route_size = false,
       tree_size = false;
};

void
compare_results (Result result1, Result result2, output info, std::ostream &out = std::cout)
{
  if (info.query)
    out << result1.query.from << " to " << result1.query.to << '\n';

  if (result1.route_found && result2.route_found)
  {
    if (info.distance)
      out << "\tdistance:  " << result1.distance << "\t" << result2.distance << '\n';
    if (info.mid_node)
      out << "\tsearches met in:  " << result1.mid_node << '\t' << result2.mid_node << '\n';
    if (info.route)
      out << "\tpath:      " << result1.route << "\n"
	  << "\tpath:      " << result2.route << '\n';
    if (info.route_size)
      out << "\tpath size: " << result1.route.nodes.size () << "\t" << result2.route.nodes.size() << '\n';
    if (info.tree_size)
      out << "\ttree size: " << result1.trees.nodes.size () << '\t' << result2.trees.nodes.size()<< '\n';
    if (info.timing)
      out << "\ttime:      " << result1.duration << "\t" << result2.duration << '\n';
  }
  else
  {
    out << "no route found" << std::endl;
  }
}

void
print_result (Result result, output info, std::ostream &out = std::cout)
{
  if (info.query)
    out << result.query.from << " to " << result.query.to << '\n';

  if (result.route_found)
  {
    if (info.distance)
      out << "\tdistance:  " << result.distance << "\n";
    if (info.mid_node)
      out << "\tsearches met in:  " << result.mid_node << '\n';
    if (info.route)
      out << "\tpath:      " << result.route << "\n";
    if (info.route_size)
      out << "\tpath size: " << result.route.nodes.size () << "\n";
    if (info.tree_size)
      out << "\ttree size: " << result.trees.nodes.size () << '\n';
    if (info.timing)
      out << "\ttime:      " << result.duration << "\n";
  }
  else
  {
    out << "no route found" << std::endl;
  }
}