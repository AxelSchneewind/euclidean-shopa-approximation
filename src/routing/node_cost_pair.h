#pragma once


template<typename NodeId, typename Distance, typename Info = void>
[[deprecated]]
struct node_cost_pair {
private:
    NodeId _node;
    NodeId _predecessor;
    Distance _distance;
    Info _info;

public:
    using info_type = Info;
    using node_id_type = NodeId;
    using distance_type = Distance;

    constexpr node_cost_pair() = default;

    constexpr node_cost_pair(NodeId node, NodeId predecessor, Distance distance, Info info)
            : _node(node), _predecessor(predecessor), _distance(distance), _info(info) {};

    node_cost_pair(NodeId node, NodeId predecessor, Distance distance)
            : _node(node), _predecessor(predecessor), _distance(distance) {};

    node_cost_pair(node_cost_pair &&) noexcept = default;

    node_cost_pair &operator=(node_cost_pair &&) noexcept = default;

    node_cost_pair(node_cost_pair const &) noexcept = default;

    node_cost_pair &operator=(node_cost_pair const &) noexcept = default;

    bool operator==(node_cost_pair<NodeId, Distance, Info> const &) const = default;

    Info& info() { return _info; }
    Info const& info() const { return _info; }

    NodeId &node() { return _node; }

    NodeId const &node() const { return _node; }

    NodeId &predecessor() { return _predecessor; }

    NodeId const &predecessor() const { return _predecessor; }

    Distance &distance() { return _distance; }

    Distance const &distance() const { return _distance; }

    auto& heuristic() { return _info; }

    auto const& heuristic() const { return _info; }

    auto& value() { return _info; }

    auto const& value() const { return _info; }
};

template<typename NodeId, typename Distance>
struct node_cost_pair<NodeId, Distance, void> {
private:
    NodeId _node;
    NodeId _predecessor;
    Distance _distance;
public:
    using info_type = void;

    node_cost_pair() = default;

    constexpr node_cost_pair(NodeId node, NodeId predecessor, Distance distance)
            : _node(node), _predecessor(predecessor), _distance(distance) {};

    node_cost_pair(node_cost_pair &&) noexcept = default;

    node_cost_pair &operator=(node_cost_pair &&) noexcept = default;

    node_cost_pair(node_cost_pair const &) noexcept = default;

    node_cost_pair &operator=(node_cost_pair const &) noexcept = default;

    bool operator==(node_cost_pair<NodeId, Distance> const &) const = default;

    NodeId &node() { return _node; }

    NodeId const &node() const { return _node; }

    NodeId &predecessor() { return _predecessor; }

    NodeId const &predecessor() const { return _predecessor; }

    Distance &distance() { return _distance; }

    Distance const &distance() const { return _distance; }

    Distance &min_distance() { return _distance; };

    Distance min_distance() const { return _distance; }

    Distance &value() { return _distance; }

    Distance const& value() const { return _distance; }
};

template<typename NodeId, typename Distance, typename Info>
constexpr node_cost_pair<NodeId, Distance, Info> optional::none_value<node_cost_pair<NodeId, Distance, Info>>
        = {none_value<NodeId>, none_value<NodeId>, infinity<Distance>, none_value<Info>};

template<typename NodeId, typename Distance>
constexpr node_cost_pair<NodeId, Distance> optional::none_value<node_cost_pair<NodeId, Distance>>
        = {none_value<NodeId>, none_value<NodeId>, infinity<Distance>};


