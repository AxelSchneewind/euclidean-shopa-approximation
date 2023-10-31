#pragma once

#include "graph.h"

#include "base_types.h"

#include <unordered_map>
#include <vector>

template <typename NodeId, typename EdgeId>
subgraph<NodeId, EdgeId>::subgraph (std::vector<NodeId> &&__n, std::vector<EdgeId> &&__e)
  : nodes (std::move (__n)), edges (std::move (__e))
{}

template <typename NodeInfo, typename EdgeInfo, typename NodeId, typename EdgeId>
graph<NodeInfo, EdgeInfo, NodeId, EdgeId>::subgraph
graph<NodeInfo, EdgeInfo, NodeId, EdgeId>::make_subgraph (std::vector<NodeId> &&nodes, std::vector<EdgeId> &&edges) const
{
  return {std::move (nodes), std::move (edges)};
}

template <typename NodeInfo, typename EdgeInfo, typename NodeId, typename EdgeId>
graph<NodeInfo, EdgeInfo, NodeId, EdgeId>::subgraph
graph<NodeInfo, EdgeInfo, NodeId, EdgeId>::make_subgraph (const graph<NodeInfo, EdgeInfo, NodeId, EdgeId>::path &route) const
{
  std::vector<node_id_t> nodes;
  std::vector<edge_id_t> edges;

  if (!route.nodes.empty ())
    nodes.push_back (route.nodes[0]);
  for (size_t i = 0; i + 1 < route.nodes.size (); i++)
    {
      auto id = route.nodes[i];
      auto id_next = route.nodes[i + 1];
      assert (id < node_count ());
      assert (id_next < node_count ());

      nodes.push_back (id_next);
      edges.push_back (_M_adjacency_list.forward ().edge_index (id, id_next));
    }

  return {std::move (nodes), std::move (edges)};
}

template <typename NodeInfo, typename EdgeInfo, typename NodeId, typename EdgeId>
graph<NodeInfo, EdgeInfo, NodeId, EdgeId>
graph<NodeInfo, EdgeInfo, NodeId, EdgeId>::make_graph (const graph<NodeInfo, EdgeInfo, NodeId, EdgeId>::subgraph &subgraph) const
{
  node_id_t node_count = subgraph.nodes.size ();
  edge_id_t edge_count = subgraph.edges.size ();

  std::vector<NodeInfo> nodes;
  std::vector<NodeId> old_node_ids;
  std::unordered_map<NodeId, NodeId> new_node_ids;

  // make node list
  for (node_id_t i = 0; i < node_count; i++)
    {
      auto id = subgraph.nodes[i];

      nodes.push_back (node (id));
      old_node_ids.push_back (id);
      new_node_ids[id] = i;
    }

  // make one-directional adjacency list
  typename unidirectional_adjacency_list<EdgeInfo>::adjacency_list_builder forward_builder;
  forward_builder.add_node (node_count - 1);
  for (edge_id_t edge : subgraph.edges)
    {
      node_id_t src = _M_adjacency_list.forward ().source (edge);
      node_id_t dest = _M_adjacency_list.forward ().destination (edge);
      EdgeInfo info = _M_adjacency_list.forward ().edge (edge);

      src = new_node_ids[src];
      dest = new_node_ids[dest];
      forward_builder.add_edge (src, dest, info);
    }

  // make bidirectional adjacency list
  auto adj_list = forward_builder.get ();
  auto l = adjacency_list<EdgeId>::make_bidirectional (std::move (adj_list));

  return graph<NodeInfo, EdgeInfo, NodeId, EdgeId>::make_graph (std::move (nodes), std::move (l));
};

template <typename NodeInfo, typename EdgeInfo, typename NodeId, typename EdgeId>
size_t
graph<NodeInfo, EdgeInfo, NodeId, EdgeId>::edge_count () const
{
  return forward ().edge_count ();
}

template <typename NodeInfo, typename EdgeInfo, typename NodeId, typename EdgeId>
size_t
graph<NodeInfo, EdgeInfo, NodeId, EdgeId>::node_count () const
{
  return _M_node_list.size ();
}

template <typename NodeInfo, typename EdgeInfo, typename NodeId, typename EdgeId>
const NodeInfo &
graph<NodeInfo, EdgeInfo, NodeId, EdgeId>::node (const node_id_t &node_id) const
{
  return _M_node_list[node_id];
}

template <typename NodeInfo, typename EdgeInfo, typename NodeId, typename EdgeId>
std::span<const NodeInfo, std::dynamic_extent>
graph<NodeInfo, EdgeInfo, NodeId, EdgeId>::nodes () const
{
  return std::span (_M_node_list.begin (), _M_node_list.end ());
}

template <typename NodeInfo, typename EdgeInfo, typename NodeId, typename EdgeId>
graph<NodeInfo, EdgeInfo, NodeId, EdgeId>::~graph ()
{
  _M_node_list.clear ();
  _M_node_list.shrink_to_fit ();
}

template <typename NodeInfo, typename EdgeInfo, typename NodeId, typename EdgeId>
graph<NodeInfo, EdgeInfo, NodeId, EdgeId>::graph (graph &&__graph) noexcept
  : _M_node_list (std::move (__graph._M_node_list)), _M_adjacency_list (std::move (__graph._M_adjacency_list))
{}

template <typename NodeInfo, typename EdgeInfo, typename NodeId, typename EdgeId>
graph<NodeInfo, EdgeInfo, NodeId, EdgeId>::graph (std::vector<NodeInfo> &&__nodes, adjacency_list<EdgeInfo> &&__list)
  : _M_node_list (std::move (__nodes)), _M_adjacency_list (std::move (__list))
{}

template <typename NodeInfo, typename EdgeInfo, typename NodeId, typename EdgeId>
graph<NodeInfo, EdgeInfo, NodeId, EdgeId>
graph<NodeInfo, EdgeInfo, NodeId, EdgeId>::make_graph (std::vector<NodeInfo> &&nodes, adjacency_list<EdgeInfo> &&forward)
{
  return graph (std::move (nodes), std::move (forward));
};
template <typename NodeInfo, typename EdgeInfo, typename NodeId, typename EdgeId>
graph<NodeInfo, EdgeInfo, NodeId, EdgeId>
graph<NodeInfo, EdgeInfo, NodeId, EdgeId>::make_graph (std::vector<NodeInfo> &&nodes,
					 const std::shared_ptr<unidirectional_adjacency_list<EdgeInfo>> &forward)
{
  auto adj_list = adjacency_list<EdgeInfo>::make_bidirectional (forward);
  return {std::move (nodes), std::move (adj_list)};
};

template<typename NodeId>
std::ostream &
operator<< (std::ostream &stream, path<NodeId> &r)
{
  stream << "{ ";

  int length = r.nodes.size ();
  if (length > 0)
    {
      for (int i = 0; i < length - 1; i++)
	stream << r.nodes[i] << ", ";
      stream << r.nodes[length - 1];
    }

  return stream << " }";
}

template <typename NodeInfo, typename EdgeInfo, typename NodeId, typename EdgeId>
const unidirectional_adjacency_list<EdgeInfo> &
graph<NodeInfo, EdgeInfo, NodeId, EdgeId>::backward () const
{
  return _M_adjacency_list.backward();
}

template <typename NodeInfo, typename EdgeInfo, typename NodeId, typename EdgeId>
const unidirectional_adjacency_list<EdgeInfo> &
graph<NodeInfo, EdgeInfo, NodeId, EdgeId>::forward () const
{
  return _M_adjacency_list.forward();
}

template <typename NodeInfo, typename EdgeInfo, typename NodeId, typename EdgeId>
graph<NodeInfo, EdgeInfo, NodeId, EdgeId>::distance_type
graph<NodeInfo, EdgeInfo, NodeId, EdgeId>::path_length (const path &route) const
{
  if (route.nodes.empty())
      return DISTANCE_INF;

  distance_t result = 0;

  for (int i = 0; i < route.nodes.size() - 1; ++i)
  {
    auto from = route.nodes[i];
    auto to = route.nodes[i + 1];
    result += forward().edge (forward().edge_index (from, to)).cost;
  }

  return result;
}
