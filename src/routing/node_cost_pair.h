#include <memory>

template<typename NodeId, typename Distance, typename Info = void>
struct node_cost_pair {
    NodeId node;
    NodeId predecessor;
    Distance distance;
    Info info;

    bool operator==(node_cost_pair<NodeId, Distance, Info> const &) const = default;

    distance_t min_distance() const { return info.min_distance(); };
};

template<typename NodeId, typename Distance>
struct node_cost_pair<NodeId, Distance, void> {
    NodeId node;
    NodeId predecessor;
    Distance distance;

    bool operator==(node_cost_pair<NodeId, Distance> const &) const = default;

    Distance min_distance() const { return distance; };
};

template<typename NodeId, typename Distance, typename Info>
node_cost_pair<NodeId, Distance, Info> none_value<node_cost_pair<NodeId, Distance, Info>> = {none_value<NodeId>,
                                                                                             none_value<NodeId>,
                                                                                             infinity<Distance>,
                                                                                             none_value<Info>};

template<typename NodeId, typename Distance>
node_cost_pair<NodeId, Distance> none_value<node_cost_pair<NodeId, Distance>> = {none_value<NodeId>, none_value<NodeId>,
                                                                                 infinity<Distance>};


