#pragma once

#include "file-io/fmi_file_io.h"
#include "file-io/formatters.h"
#include "file-io/gl_file_io.h"
#include "file-io/triangulation_file_io.h"
#include "graph/adjacency_list.h"
#include "graph/base_types.h"
#include "graph/graph.h"
#include "graph/unidirectional_adjacency_list.h"
#include "routing/dijkstra.h"
#include "routing/node_labels.h"
#include "routing/router.h"

#include "file-io/fmi_file_io_impl.h"
#include "file-io/formatters_impl.h"
#include "file-io/gl_file_io_impl.h"
#include "file-io/triangulation_file_io_impl.h"
#include "graph/adjacency_list_impl.h"
#include "graph/base_types_impl.h"
#include "graph/graph_impl.h"
#include "graph/unidirectional_adjacency_list_impl.h"
#include "routing/dijkstra_impl.h"
#include "routing/router_impl.h"
#include "triangulation/steiner_graph_impl.h"
#include "triangulation/steiner_labels.h"
#include "triangulation/steiner_labels_impl.h"

using std_graph_t = graph<node_t, edge_t, node_id_t, edge_id_t>;

template <RoutableGraph G> struct label_type
{
  G::distance_type distance;
  G::node_id_type predecessor;
};

template <>
constexpr label_type<steiner_graph>
none_value ()
{
  return { infinity<steiner_graph::distance_type> (), none_value<steiner_graph::node_id_type> () };
}

struct a_star_info
{
  // value from a* heuristic (distance + minimal remaining distance)
  distance_t value;
};
using steiner_a_star_node_cost_pair = node_cost_pair<steiner_graph, a_star_info>;

using steiner_queue_t = a_star_queue<steiner_graph, steiner_a_star_node_cost_pair>;
using steiner_labels_t = steiner_labels<steiner_graph, label_type<steiner_graph>>;
using steiner_dijkstra = dijkstra<steiner_graph, steiner_queue_t, use_all_edges<steiner_graph>, steiner_labels_t>;
using steiner_routing_t = router<steiner_graph, steiner_dijkstra>;

template <>
void
steiner_dijkstra::expand (steiner_dijkstra::node_cost_pair __node)
{
  // static std::vector<internal_adjacency_list_edge<steiner_graph::node_id_type, steiner_graph::edge_info_type>> edges;
  // TODO
  fixed_capacity_vector<internal_adjacency_list_edge<steiner_graph::node_id_type, steiner_graph::edge_info_type>, 100>
      edges;
  fixed_capacity_vector<coordinate_t, 100> destination_coordinates;
  assert (!is_none (__node.node));

  // get inverse edge for current base edge
  auto inv_edge = _M_graph->base_polyhedron ().inverse_edge (__node.node.edge);

  // get triangles that have not been visited yet
  auto triangles = _M_graph->base_polyhedron ().edge_faces (__node.node.edge);
  char triangle_first = 0;
  char triangle_last = 2;

  // if this node is reached via a face crossing segment, only use edges in next face
  if (__node.predecessor.edge != __node.node.edge && __node.predecessor.edge != inv_edge) [[likely]]
  {
    auto visited_triangles = _M_graph->base_polyhedron ().edge_faces (__node.predecessor.edge);

    if (is_none (triangles[0]) || triangles[0] == visited_triangles[0] || triangles[0] == visited_triangles[1])
	[[unlikely]]
    {
      triangle_first = 1;
    }
    if (is_none (triangles[1]) || triangles[1] == visited_triangles[0] || triangles[1] == visited_triangles[1])
	[[unlikely]]
    {
      triangle_last = 1;
    }
  }
  else
  {
    if (is_none (triangles[0])) [[unlikely]]
    {
      triangle_first = 1;
    }
    if (is_none (triangles[1])) [[unlikely]]
    {
      triangle_last = 1;
    }
  }

  // make list of edges (i.e. destination/cost pairs)
  coordinate_t source_coordinate = _M_graph->node (__node.node).coordinates;
  auto steiner_info = _M_graph->steiner_info (__node.node.edge);

  // for inverse edge
  if (__node.node.steiner_index == steiner_info.node_count - 1) [[unlikely]]
  {
    steiner_graph::node_id_type destination = { inv_edge, _M_graph->steiner_info (inv_edge).node_count - 1 };
    coordinate_t destination_coordinate = _M_graph->node (destination).coordinates;
    edges.push_back ({ destination, {} });
    destination_coordinates.push_back (destination_coordinate);
  }

  // for neighboring node on own edge
  if (__node.node.steiner_index < steiner_info.node_count - 1) [[likely]]
  {
    steiner_graph::node_id_type destination = { __node.node.edge, __node.node.steiner_index + 1 };
    coordinate_t destination_coordinate = _M_graph->node (destination).coordinates;
    edges.push_back ({ destination, {} });
    destination_coordinates.push_back (destination_coordinate);
  }

  if (__node.node.steiner_index > 0) [[likely]]
  {
    steiner_graph::node_id_type destination = { __node.node.edge, __node.node.steiner_index - 1 };
    coordinate_t destination_coordinate = _M_graph->node (destination).coordinates;
    edges.push_back ({ destination, {} });
    destination_coordinates.push_back (destination_coordinate);
  }

  // face-crossing edges
  for (char triangle_index = triangle_first; triangle_index < triangle_last; triangle_index++) [[unlikely]]
  {
    assert (!is_none (triangles[triangle_index]));
    auto triangle_edges = _M_graph->base_polyhedron ().face_edges (triangles[triangle_index]);

    for (auto base_edge_id : triangle_edges) [[likely]]
    {
      if (base_edge_id == __node.node.edge || base_edge_id == inv_edge) [[unlikely]]
	continue;

      auto destination_steiner_info = _M_graph->steiner_info (base_edge_id);

      for (int i = 0; i < destination_steiner_info.node_count; ++i) [[likely]]
      {
	steiner_graph::node_id_type destination = { base_edge_id, i };
	coordinate_t destination_coordinate = _M_graph->node (destination).coordinates;
	edges.push_back ({ destination, {} });
	destination_coordinates.push_back (destination_coordinate);
      }
    }
  }

  assert (edges.size () <= 100);

  // compute distances (can be vectorized)
  for (int e = 0; e < edges.size (); ++e) [[likely]]
  {
    edges[e].info.cost = distance (source_coordinate, destination_coordinates[e]) + __node.distance;
  }

  for (auto edge : edges)
  {
    // ignore certain edges
    if (!_M_use_edge (__node.node, edge)) [[unlikely]]
    {
      continue;
    }

    assert (_M_graph->has_edge (__node.node, edge.destination));

    assert (!is_none (edge.destination));
    assert (_M_graph->has_edge (__node.node, edge.destination));

    const steiner_graph::node_id_type &successor = edge.destination;
    const distance_t successor_cost = _M_labels.get (successor).distance;

    if (edge.info.cost < successor_cost) [[unlikely]]
    {
      // (re-)insert node into the queue with updated priority
      _M_queue.push (successor, __node.node, edge.info.cost);
    }
  }

  edges.clear ();
}
