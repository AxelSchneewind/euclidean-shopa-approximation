#pragma once

#include "base_types.h"
#include "unidirectional_adjacency_list.h"
#include "adjacency_list.h"
#include <unordered_map>
#include <vector>

template<typename NodeId>
struct path
{
  std::vector<NodeId> nodes;
};

template<typename NodeId, typename EdgeId>
struct subgraph
{
  std::vector<NodeId> nodes;
  std::vector<EdgeId> edges;

  subgraph () = default;
  subgraph (std::vector<NodeId> &&__n, std::vector<EdgeId> &&__e);
};

template <typename NodeInfo, typename EdgeInfo, typename NodeId, typename EdgeId> class graph
{
public:
  using node_info_type = NodeInfo;
  using edge_info_type = EdgeInfo;
  using path = path<NodeId>;
  using subgraph = subgraph<NodeId, EdgeId>;

private:
  // node data
  std::vector<NodeInfo> m_node_list; // accessed via node_id_t

  // topology of forward and backward graphs
  adjacency_list<EdgeInfo> m_list;

  // constructors
  graph (std::vector<NodeInfo> &&__nodes, adjacency_list<EdgeInfo> &&__list);

  // move constructor
  graph (graph &&__graph) noexcept;
  graph (const graph<NodeInfo, EdgeInfo, NodeId, EdgeId> &other) = default;

  graph<NodeInfo, EdgeInfo, NodeId, EdgeId> &operator= (const graph<NodeInfo, EdgeInfo, NodeId, EdgeId> &) = default;
  graph<NodeInfo, EdgeInfo, NodeId, EdgeId> &operator= (graph<NodeInfo, EdgeInfo, NodeId, EdgeId> &&) = default;

public:
  // destructor
  ~graph ();

  static graph make_graph (std::vector<NodeInfo> &&nodes, adjacency_list<EdgeInfo> &&forward);
  static graph make_graph (std::vector<NodeInfo> &&nodes,
			     const std::shared_ptr<unidirectional_adjacency_list<EdgeInfo>> &forward);

  inline std::span<const NodeInfo> nodes () const;

  [[nodiscard]] inline node_id_t node_count () const;

  [[nodiscard]] inline edge_id_t edge_count () const;

  inline const NodeInfo &node (const node_id_t &node_id) const;

  [[nodiscard]] inline const coordinate_t &coordinates (const node_id_t &node_id) const;

  // TODO remove
  [[deprecated]] inline const adjacency_list<EdgeInfo> &get_list () const;

  inline const unidirectional_adjacency_list<EdgeInfo> &forward () const;

  inline const unidirectional_adjacency_list<EdgeInfo> &backward () const;

  distance_t path_length (const path &route) const;

  subgraph make_subgraph (const path &route) const;
  subgraph make_subgraph (std::vector<node_id_t> &&nodes, std::vector<edge_id_t> &&edges) const;

  graph make_graph (const subgraph &subgraph) const;
};


template<typename Nid>
std::ostream &
operator<< (std::ostream &stream, path<Nid> &r);

