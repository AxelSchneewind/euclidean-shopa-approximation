#pragma once

#include "graph.h"

#include "base_types.h"

#include <unordered_map>
#include <vector>

subgraph::subgraph (std::vector<node_id_t> &&__n, std::vector<edge_id_t> &&__e)
  : nodes (std::move (__n)), edges (std::move (__e))
{}

template <typename NodeInfo, typename EdgeInfo>
subgraph
graph<NodeInfo, EdgeInfo>::make_subgraph (std::vector<node_id_t> &&nodes, std::vector<edge_id_t> &&edges) const
{
  return {std::move (nodes), std::move (edges)};
}

template <typename NodeInfo, typename EdgeInfo>
subgraph
graph<NodeInfo, EdgeInfo>::make_subgraph (const path &route) const
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
      edges.push_back (m_list.forward ().edge_index (id, id_next));
    }

  return {std::move (nodes), std::move (edges)};
}

template <typename N, typename E>
graph<N, E>
graph<N, E>::make_graph (const subgraph &subgraph) const
{
  node_id_t node_count = subgraph.nodes.size ();
  edge_id_t edge_count = subgraph.edges.size ();

  std::vector<N> nodes;
  std::vector<node_id_t> old_node_ids;
  std::unordered_map<node_id_t, node_id_t> new_node_ids;

  // make node list
  for (node_id_t i = 0; i < node_count; i++)
    {
      auto id = subgraph.nodes[i];

      nodes.push_back (node (id));
      old_node_ids.push_back (id);
      new_node_ids[id] = i;
    }

  // make one-directional adjacency list
  typename unidirectional_adjacency_list<E>::adjacency_list_builder forward_builder;
  forward_builder.add_node (node_count - 1);
  for (edge_id_t edge : subgraph.edges)
    {
      node_id_t src = m_list.forward ().source (edge);
      node_id_t dest = m_list.forward ().destination (edge);
      E info = m_list.forward ().edge (edge);

      src = new_node_ids[src];
      dest = new_node_ids[dest];
      forward_builder.add_edge (src, dest, info);
    }

  // make bidirectional adjacency list
  auto adj_list = forward_builder.get ();
  auto l = adjacency_list<E>::make_bidirectional (std::move (adj_list));

  return graph<N, E>::make_graph (std::move (nodes), std::move (l));
};

template <typename NodeInfo, typename EdgeInfo>
edge_id_t
graph<NodeInfo, EdgeInfo>::edge_count () const
{
  return forward ().edge_count ();
}

template <typename NodeInfo, typename EdgeInfo>
node_id_t
graph<NodeInfo, EdgeInfo>::node_count () const
{
  return m_node_list.size ();
}

template <typename NodeInfo, typename EdgeInfo>
const NodeInfo &
graph<NodeInfo, EdgeInfo>::node (const node_id_t &node_id) const
{
  return m_node_list[node_id];
}

template <typename NodeInfo, typename EdgeInfo>
std::span<const NodeInfo, std::dynamic_extent>
graph<NodeInfo, EdgeInfo>::nodes () const
{
  return std::span (m_node_list.begin (), m_node_list.end ());
}

template <typename NodeInfo, typename EdgeInfo>
const coordinate_t &
graph<NodeInfo, EdgeInfo>::coordinates (const node_id_t &node_id) const
{
  return node (node_id).coordinates;
}

template <typename NodeInfo, typename EdgeInfo>
const adjacency_list<EdgeInfo> &
graph<NodeInfo, EdgeInfo>::get_list () const
{
  return m_list;
}

template <typename NodeInfo, typename EdgeInfo> graph<NodeInfo, EdgeInfo>::~graph ()
{
  m_node_list.clear ();
  m_node_list.shrink_to_fit ();
}

template <typename NodeInfo, typename EdgeInfo>
graph<NodeInfo, EdgeInfo>::graph (graph &&__graph) noexcept
  : m_node_list (std::move (__graph.m_node_list)), m_list (std::move (__graph.m_list))
{}

template <typename NodeInfo, typename EdgeInfo>
graph<NodeInfo, EdgeInfo>::graph (std::vector<NodeInfo> &&__nodes, adjacency_list<EdgeInfo> &&__list)
  : m_node_list (std::move (__nodes)), m_list (std::move (__list))
{}

template <typename NodeInfo, typename EdgeInfo>
graph<NodeInfo, EdgeInfo>
graph<NodeInfo, EdgeInfo>::make_graph (std::vector<NodeInfo> &&nodes, adjacency_list<EdgeInfo> &&forward)
{
  return graph (std::move (nodes), std::move (forward));
};
template <typename NodeInfo, typename EdgeInfo>
graph<NodeInfo, EdgeInfo>
graph<NodeInfo, EdgeInfo>::make_graph (std::vector<NodeInfo> &&nodes,
					 const std::shared_ptr<unidirectional_adjacency_list<EdgeInfo>> &forward)
{
  auto adj_list = adjacency_list<EdgeInfo>::make_bidirectional (forward);
  return {std::move (nodes), std::move (adj_list)};
};

std::ostream &
operator<< (std::ostream &stream, path &r)
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

template <typename NodeInfo, typename EdgeInfo>
const unidirectional_adjacency_list<EdgeInfo> &
graph<NodeInfo, EdgeInfo>::backward () const
{
  return m_list.backward();
}

template <typename NodeInfo, typename EdgeInfo>
const unidirectional_adjacency_list<EdgeInfo> &
graph<NodeInfo, EdgeInfo>::forward () const
{
  return m_list.forward();
}

template <typename NodeInfo, typename EdgeInfo>
distance_t
graph<NodeInfo, EdgeInfo>::path_length (const path &route) const
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
