#pragma once

#include "../graph/adjacency_list.h"
#include "../graph/base_types.h"
#include "../graph/graph.h"
#include "../graph/unidirectional_adjacency_list.h"

#include <queue>
#include <vector>

#include <cmath>
#include <concepts>
#include <iostream>

template <typename Info = std::nullptr_t> struct node_cost_pair
{
  node_id_t node;
  node_id_t predecessor;
  distance_t distance;
  Info info;
};

template <> struct node_cost_pair<std::nullptr_t>
{
  node_id_t node;
  node_id_t predecessor;
  distance_t distance;
};

template <typename Graph, typename Queue, typename UseEdge> class dijkstra
{
public:
  using type = dijkstra<Graph, Queue, UseEdge>;
  using node_cost_pair = Queue::value_type;
  using edge_info_type = Graph::edge_info_type;

  // determines optimality of labels depending on whether the graph allows shortcuts
  // TODO find more elegant way for this
  static constexpr bool search_symmetric = (typeid (edge_info_type) != typeid (ch_edge_t));

  // TODO move to own file
  class Labels
  {
  private:
    dijkstra<Graph, Queue, UseEdge> *d;

    std::vector<node_id_t> _M_visited;
    std::vector<distance_t> _M_distance;
    std::vector<node_id_t> _M_predecessor;

  public:
    explicit Labels (dijkstra<Graph, Queue, UseEdge> *d);

    // init for given query
    void init (node_id_t start_node, node_id_t target_node);

    bool reached (const node_id_t &node) const;
    const distance_t &distance (const node_id_t &node) const;
    const node_id_t &predecessor (const node_id_t &node) const;

    std::span<const node_id_t> all_visited () const;

    // label node with distance and predecessor
    void label (const node_cost_pair &node_cost_pair);
  };

private:
  std::shared_ptr<const Graph> _M_graph;
  adjacency_list<edge_info_type> _M_adj_list;

  node_id_t _M_start_node;
  node_id_t _M_target_node;

  Queue _M_queue;
  UseEdge _M_use_edge;

  Labels _M_labels;

  // add reachable (and not settled) nodes to active nodes in queue
  void expand (const node_cost_pair &__node);

  dijkstra (dijkstra &&__other) noexcept;
  dijkstra (const dijkstra &__other) = default;

public:
  // constructs a dijkstra object for the given graph and m_adj_list
  explicit dijkstra (const std::shared_ptr<const Graph> &__graph, const adjacency_list<edge_info_type> &__adj_list);
  ~dijkstra () = default;

  dijkstra<Graph, Queue, UseEdge> &operator= (dijkstra<Graph, Queue, UseEdge> &&other) = default;
  dijkstra<Graph, Queue, UseEdge> &operator= (const dijkstra<Graph, Queue, UseEdge> &other) = default;

  node_id_t source () const { return _M_start_node; }
  node_id_t target () const { return _M_target_node; }

  const Labels &labels () const { return _M_labels; }

  /**
   * init one to one
   *
   * @param start_node
   * @param target_node
   */
  void init (node_id_t start_node, node_id_t target_node = NO_NODE_ID);

  /**
   * get the current node without removing from queue
   * @return
   */
  const node_cost_pair &current () const;

  /**
   * step to current node in queue, i.e. store label and add adjacent nodes
   */
  void step ();

  /**
   * function to check whether search is finished (queue empty)
   * @return
   */
  bool queue_empty () const;

  /**
   * check if node is reached (labelled)
   * @param node
   * @return
   */
  bool reached (const node_id_t &node) const;
};
