#pragma once

#include "../routing.h"
#include <array>
#include <cassert>
#include <chrono>
#include <fstream>
#include <iostream>

#define assert_equal(actual, expected)                                                                                 \
  {                                                                                                                    \
    if (expected != actual)                                                                                            \
    {                                                                                                                  \
      std::cout << "expected " << expected << ", but got " << actual << std::endl;                                     \
      assert (expected == actual);                                                                                     \
    }                                                                                                                  \
  }

struct Query
{
  node_id_t from;
  node_id_t to;
};

std::vector<Query>
get_queries (std::ifstream &input)
{
  std::vector<Query> result;
  stream_encoders::encode_text f;
  while (input)
  {
    auto s = f.read<node_id_t> (input);
    auto t = f.read<node_id_t> (input);
    result.push_back ({ s, t });
  }

  return result;
}

std::vector<distance_t>
get_distances (std::ifstream &input)
{
  std::vector<distance_t> result;
  stream_encoders::encode_text f;
  while (input)
  {
    auto d = f.read<distance_t> (input);
    result.push_back (d);
  }

  return result;
}

template <typename Graph, typename Router>
void
test_routing (const Graph &graph, Router &router, const std::vector<Query> &queries,
	      const std::vector<distance_t> &expected)
{
  for (int i = 0; i < queries.size (); i++)
  {
    node_id_t from = queries[i].from;
    node_id_t to = queries[i].to;

    std::cout << "routing from " << from << " to " << to << ": " << std::flush;

    router.compute_route (from, to);
    if (router.route_found ())
    {
      path route = router.route ();

      std::cout << "path found " << route << std::endl;

      assert_equal (expected[i], router.distance ());
    }
    else
      std::cout << "no path found " << std::endl;

    std::cout << "done" << std::endl;
  }
};

template <typename Graph, typename Router>
void
test_routing (const Graph &graph, Router &router, const std::vector<Query> &queries)
{
  for (int i = 0; i < queries.size (); i++)
  {
    node_id_t from = queries[i].from;
    node_id_t to = queries[i].to;

    std::cout << "routing from " << from << " to " << to << ": " << std::flush;

    // setup timing
    std::chrono::time_point<std::chrono::system_clock, std::chrono::duration<double>> before
      = std::chrono::high_resolution_clock::now ();

    router.compute_route (from, to);

    std::chrono::time_point<std::chrono::system_clock, std::chrono::duration<double>> after
      = std::chrono::high_resolution_clock::now ();
    std::chrono::duration<double, std::milli> routing_time = after - before;

    if (router.route_found ())
    {
      path route = router.route ();
      subgraph tree = router.shortest_path_tree ();

      std::cout << " done"
		<< "\n\ttime:      " << routing_time
		<< "\n\tdistance:  " << graph.path_length (route)
		<< "\n\tpath:      " << route
		<< "\n\tmid node:  " << router.mid_node ()
		<< "\n\ttree size: " << tree.nodes.size () << std::endl;
    }
    else
    {
      std::cout << "no route found" << std::endl;
    }
  }
}

template <typename N, typename E>
bool
check_graph (const graph<N, E> &graph)
{
  bool correct = true;
  for (edge_id_t e = 0; e < graph.edge_count (); ++e)
  {
    auto src0 = graph.forward ().source (e);
    auto dest0 = graph.forward ().destination (e);

    auto src0_inv = graph.backward_node_id (src0);
    auto dest0_inv = graph.backward_node_id (dest0);

    auto e_inv = graph.backward ().edge_index (dest0_inv, src0_inv);
    auto dest1 = graph.backward ().destination (e_inv);
    auto src1 = graph.backward ().source (e_inv);

    auto src1_inv = graph.forward_node_id (src1);
    auto dest1_inv = graph.forward_node_id (dest1);

    if (src0 != dest1_inv)
    {
      correct = false;
      std::cout << " edge (" << src0 << "," << dest0 << ") has no equivalent in backward adjacency list" << std::endl;
    }
  }

  // check_graph linking of nodes in forward and backward adjacency lists
  for (node_id_t n = 0; n < graph.node_count (); ++n)
  {
    auto forward_id = n;
    auto backward_id = graph.backward_node_id (n);
    auto forward_id1 = graph.forward_node_id (backward_id);

    if (forward_id != forward_id1)
    {
      correct = false;
      std::cout << " node (" << forward_id << " does not match a node in the backward m_adj_list" << std::endl;
    }
  }

  return correct;
}

template <typename edge>
void
assert_adjacency_list_equal (const unidirectional_adjacency_list<edge> &list, size_t expected_node_count,
			     size_t expected_edge_count, std::vector<int> &expected_offsets,
			     std::vector<adjacency_list_edge<edge>> &expected_edges);

// TODO return bool
template <typename edge>
void
assert_adjacency_list_equal (const unidirectional_adjacency_list<edge> &list, size_t expected_node_count,
			     size_t expected_edge_count, std::vector<int> &expected_offsets,
			     std::vector<adjacency_list_edge<edge>> &expected_edges)
{
  assert_equal (list.node_count (), expected_node_count);
  assert_equal (list.edge_count (), expected_edge_count);

  for (size_t node_index = 0; node_index < expected_offsets.size (); node_index++)
  {
    assert_equal (list.offset (node_index), expected_offsets[node_index]);
  }

  for (size_t edge_index = 0; edge_index < expected_edges.size (); edge_index++)
  {
    assert_equal (list.destination (edge_index), expected_edges[edge_index].destination);
    assert_equal (list.source (edge_index), expected_edges[edge_index].source);
    assert_equal (list.edge (edge_index).cost, expected_edges[edge_index].info.cost);
  }
}

template <typename N, typename E>
void
test_routes (routing<N, E> *router, const std::vector<node_id_t> &sources, const std::vector<node_id_t> &targets,
	     const std::vector<distance_t> &distances);
template <typename N, typename E>
void
test_routes (routing<N, E> *router, const std::vector<node_id_t> &sources, const std::vector<node_id_t> &targets,
	     const std::vector<distance_t> &distances)
{
  for (int i = 0; i < sources.size (); i++)
  {
    node_id_t from = sources[i];
    node_id_t to = targets[i];
    router->compute_route (from, to);
    assert_equal (distances[i], router->distance ());
  }
}

template <typename Reader, typename node_t, typename edge_t, typename Check>
bool
test_read (const std::string &filename, Check check)
{
  std::cout << "reading from " << filename << ": " << std::endl;
  std::ifstream input;
  input.open (filename);

  if (input.bad () || input.eof ())
    std::cout << " bad file" << std::endl;

  Reader reader;
  graph<node_t, edge_t> graph (reader.template read<node_t, edge_t> (input));

  bool success = check (graph);

  if (success)
    std::cout << " successful" << std::endl;
  return success;
}
