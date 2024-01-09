#include <memory>

template <typename NodeId, typename Distance, typename Info = void> struct node_cost_pair
{
  NodeId node;
  NodeId predecessor;
  Distance distance;
  Info info;

  using info_type = Info;

  node_cost_pair () = default;
  node_cost_pair (NodeId node, NodeId predecessor, Distance distance, Info info)
    : node (node), predecessor (predecessor), distance (distance), info (info){};
  node_cost_pair (NodeId node, NodeId predecessor, Distance distance)
    : node (node), predecessor (predecessor), distance (distance){};

  node_cost_pair (node_cost_pair &&) noexcept = default;
  node_cost_pair &operator= (node_cost_pair &&) noexcept = default;

  node_cost_pair (node_cost_pair const &) noexcept = default;
  node_cost_pair &operator= (node_cost_pair const &) noexcept = default;

  bool operator== (node_cost_pair<NodeId, Distance, Info> const &) const = default;

  Distance min_distance () const { return info.min_distance (); };

  Distance value () const { return info.value (); }
};

template <typename NodeId, typename Distance> struct node_cost_pair<NodeId, Distance, void>
{
  NodeId node;
  NodeId predecessor;
  Distance distance;

  using info_type = void;

  node_cost_pair () = default;
  node_cost_pair (NodeId node, NodeId predecessor, Distance distance)
    : node (node), predecessor (predecessor), distance (distance){};

  node_cost_pair (node_cost_pair &&) noexcept = default;
  node_cost_pair &operator= (node_cost_pair &&) noexcept = default;

  node_cost_pair (node_cost_pair const &) noexcept = default;
  node_cost_pair &operator= (node_cost_pair const &) noexcept = default;

  bool operator== (node_cost_pair<NodeId, Distance> const &) const = default;

  Distance min_distance () const { return distance; };

  Distance value () const { return distance; }
};

template <typename NodeId, typename Distance, typename Info>
node_cost_pair<NodeId, Distance, Info> none_value<node_cost_pair<NodeId, Distance, Info>>
    = { none_value<NodeId>, none_value<NodeId>, infinity<Distance>, none_value<Info> };

template <typename NodeId, typename Distance>
node_cost_pair<NodeId, Distance> none_value<node_cost_pair<NodeId, Distance>>
    = { none_value<NodeId>, none_value<NodeId>, infinity<Distance> };
