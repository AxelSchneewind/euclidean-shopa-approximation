#pragma once

#include "dijkstra_impl.h"
#include <concepts>
#include <tuple>


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


template <RoutableGraph Graph> struct use_all_edges
{
protected:
  const Graph *g;

public:
  use_all_edges (const Graph *g) : g (g) {}

  bool operator() (const node_id_t &node, const adjacency_list_edge<typename Graph::edge_info_type> &via)
  {
    return true;
  };
};

template <RoutableGraph Graph> struct use_upward_edges
{
protected:
  const Graph *g;

public:
  use_upward_edges (const Graph *g) : g (g) {}

  bool operator() (const node_id_t &node, const adjacency_list_edge<typename Graph::edge_info_type> &via)
  {
    return g->node(node).level <= g->node(via.destination).level;
  };
};

template <typename NodeCostPair> struct Default
{
public:
  Default (){};

  bool operator() (const NodeCostPair &n1, const NodeCostPair &n2) { return n1.distance > n2.distance; };
};

template <typename NodeCostPair> struct A_Star
{
public:
  A_Star () {};

  bool operator() (const NodeCostPair &__n1, const NodeCostPair &__n2)
  {
    return __n1.info.value > __n2.info.value;
  };
};

template <RoutableGraph Graph, typename NodeCostPair, typename Comp>
class dijkstra_queue : protected std::priority_queue<NodeCostPair, std::vector<NodeCostPair>, Comp>
{
protected:
public:
  using value_type = NodeCostPair;

  dijkstra_queue (std::shared_ptr<const Graph> __graph, Comp __comp = Comp{}) {}

  virtual void init (node_id_t __start_node, node_id_t __target_node){
    while (!empty())
      pop();
  };

  virtual void push (node_id_t __node, node_id_t __predecessor, distance_t __dist)
  {
    NodeCostPair ncp (__node, __predecessor, __dist);
    std::priority_queue<NodeCostPair, std::vector<NodeCostPair>, Comp>::push (ncp);
  }

  Comp &get_comp () { return this->std::priority_queue<NodeCostPair, std::vector<NodeCostPair>, Comp>::comp; }

  bool empty () const { return this->std::priority_queue<NodeCostPair, std::vector<NodeCostPair>, Comp>::empty (); }

  void pop () { return this->std::priority_queue<NodeCostPair, std::vector<NodeCostPair>, Comp>::pop (); }

  const NodeCostPair &top () const
  {
    return this->std::priority_queue<NodeCostPair, std::vector<NodeCostPair>, Comp>::top ();
  }
};

template <typename Graph, typename NodeCostPair, typename Comp = A_Star<NodeCostPair>>
class a_star_queue : public dijkstra_queue<Graph, NodeCostPair, Comp>
{
private:
  std::shared_ptr<const Graph> _M_graph;
  coordinate_t _M_target_coordinates;

public:
  using value_type = NodeCostPair;

  a_star_queue (std::shared_ptr<const Graph> __graph, Comp __comp = Comp{})
    : dijkstra_queue<Graph, NodeCostPair, Comp> (__graph, __comp), _M_graph (__graph)
  {}

  void push (node_id_t __node, node_id_t __predecessor, distance_t __dist) override
  {
    NodeCostPair ncp (__node, __predecessor, __dist);
    ncp.info.value = __dist + distance (_M_target_coordinates, _M_graph->node (__node).coordinates);
    std::priority_queue<NodeCostPair, std::vector<NodeCostPair>, Comp>::push (ncp);
  }

  void init (node_id_t /*__start_node*/, node_id_t __target_node) override
  {
    while (!dijkstra_queue<Graph, NodeCostPair, Comp>::empty())
      dijkstra_queue<Graph, NodeCostPair, Comp>::pop();

    _M_target_coordinates = _M_graph->node (__target_node).coordinates;
  }
};
