#pragma once

#include "../routing/dijkstra_concepts.h"
#include "../util/optional.h"

template<typename NodeId, typename Distance, typename Heuristic, typename BendingPoint>
struct geometric_node_cost_pair;

template<typename NodeId, typename Distance, typename BendingPoint>
struct geometric_node_cost_pair<NodeId, Distance, void, BendingPoint> {
    using predecessor_node_id_type = BendingPoint;
    using node_id_type = NodeId;
    using distance_type = Distance;

private:
    node_id_type _node;
    node_id_type _predecessor;

    distance_type _distance;

    predecessor_node_id_type _face_crossing_predecessor;

public:

    constexpr geometric_node_cost_pair() : _node{optional::none_value<node_id_type>}, _predecessor{}, _distance{}, _face_crossing_predecessor{} {};

    constexpr geometric_node_cost_pair(node_id_type node, node_id_type predecessor, distance_type distance)
            : _node{node}, _predecessor{ predecessor }, _distance{ distance}, _face_crossing_predecessor{} {};

    constexpr geometric_node_cost_pair(node_id_type node, node_id_type predecessor, distance_type distance,
                                       predecessor_node_id_type bending_point)
    : _node{node}, _predecessor{ predecessor }, _distance{ distance}, _face_crossing_predecessor{ bending_point} {};

    geometric_node_cost_pair(geometric_node_cost_pair &&) noexcept = default;

    geometric_node_cost_pair &operator=(geometric_node_cost_pair &&) noexcept = default;

    geometric_node_cost_pair(geometric_node_cost_pair const &) noexcept = default;

    geometric_node_cost_pair &operator=(geometric_node_cost_pair const &) noexcept = default;

    bool operator==(geometric_node_cost_pair const &) const = default;

    node_id_type &node() { return _node; }

    node_id_type const &node() const { return _node; }

    node_id_type &predecessor() { return _predecessor; }

    node_id_type const &predecessor() const { return _predecessor; }

    distance_type &distance() { return _distance; }

    distance_type const &distance() const { return _distance; }

    distance_type &value() { return _distance; }

    distance_type const &value() const { return _distance; }

    predecessor_node_id_type &face_crossing_predecessor() { return _face_crossing_predecessor; }

    predecessor_node_id_type const &face_crossing_predecessor() const { return _face_crossing_predecessor; }
};

template<typename NodeId, typename Distance, typename Heuristic, typename BendingPoint> requires requires {!std::same_as<Heuristic, void>;}
struct geometric_node_cost_pair <NodeId, Distance, Heuristic, BendingPoint> : public geometric_node_cost_pair<NodeId, Distance, void, BendingPoint> {
private:
    using predecessor_node_id_type = BendingPoint;
    using node_id_type = NodeId;
    using distance_type = Distance;
    using heuristic_type = Heuristic;

    heuristic_type _heuristic;

public:
    constexpr geometric_node_cost_pair()
        : geometric_node_cost_pair<NodeId, Distance, void, BendingPoint>{}, _heuristic{} {};

    constexpr geometric_node_cost_pair(node_id_type node, node_id_type predecessor, distance_type distance)
            : geometric_node_cost_pair<NodeId, Distance, void, BendingPoint>{node, predecessor, distance}, _heuristic(distance) {};

    constexpr geometric_node_cost_pair(node_id_type node, node_id_type predecessor, distance_type distance,
                                       heuristic_type heuristic, predecessor_node_id_type bending_point)
            : geometric_node_cost_pair<NodeId, Distance, void, BendingPoint>{node, predecessor, distance, bending_point}, _heuristic{ heuristic} {};

    heuristic_type &heuristic() { return _heuristic; }

    heuristic_type const &heuristic() const { return _heuristic; }

    heuristic_type &value() { return _heuristic; }

    heuristic_type const &value() const { return _heuristic; }
};

namespace optional {
    template<typename NodeId, typename Distance, typename Heuristic, typename BendingPoint>
    constexpr geometric_node_cost_pair<NodeId, Distance, Heuristic, BendingPoint> none_value<geometric_node_cost_pair<NodeId, Distance, Heuristic, BendingPoint>> = geometric_node_cost_pair<NodeId, Distance, Heuristic, BendingPoint>();

    template<typename NodeId, typename Distance, typename Heuristic, typename BendingPoint>
    constexpr bool is_none(geometric_node_cost_pair<NodeId, Distance, Heuristic, BendingPoint> p) { return is_none(p.node()); }
}

