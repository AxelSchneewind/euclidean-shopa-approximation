#pragma once

#include "../graph/base_types.h"

#include <concepts>
#include <cstddef>

#include <numbers>
#include <cmath>
#include <vector>

template<typename Label, typename Graph>
concept HasSuccessorHint = requires {
    typename Graph::node_id_type;
} && requires(Label &n) {
    { n.successor_hint() } -> std::convertible_to<typename Graph::node_id_type &>;
} && requires(Label const &n) {
    { n.successor_hint() } -> std::convertible_to<typename Graph::node_id_type>;
};

template<typename NodeCostPair, typename Graph>
concept HasFaceCrossingPredecessor = requires {
    typename Graph::node_id_type;
} && requires( NodeCostPair &n) {
    { n.face_crossing_predecessor() } -> std::convertible_to<typename Graph::node_id_type &>;
} && requires( NodeCostPair const &n) {
    { n.face_crossing_predecessor() } -> std::convertible_to<typename Graph::node_id_type>;
};

template<typename NodeCostPair, typename Graph>
struct geometric_node_cost_pair : public NodeCostPair {
private:
    typename Graph::node_id_type _face_crossing_predecessor;
public:
    constexpr geometric_node_cost_pair() = default;

    constexpr geometric_node_cost_pair(const NodeCostPair &other, typename Graph::node_id_type n) : NodeCostPair(other),
                                                                                                    _face_crossing_predecessor{n} {};

    template<typename ...Args>
    geometric_node_cost_pair(Args... args) : NodeCostPair(args...), _face_crossing_predecessor{this->predecessor()} {};


    typename Graph::node_id_type &face_crossing_predecessor() { return _face_crossing_predecessor; }

    typename Graph::node_id_type const &face_crossing_predecessor() const { return _face_crossing_predecessor; }

    static_assert(HasFaceCrossingPredecessor<geometric_node_cost_pair, Graph>);
};

template<typename NodeCostPair, typename Graph>
constexpr geometric_node_cost_pair<NodeCostPair, Graph> none_value<geometric_node_cost_pair<NodeCostPair, Graph>> = {
        none_value<NodeCostPair>, none_value<typename Graph::node_id_type>};


template<typename Graph, typename Labels>
struct steiner_neighbors {
private:
    Graph const &_graph;
    Labels &_labels;

    double _spanner_angle;
    double _spanner_angle_cos;

    double _max_angle;
    double _max_angle_cos;

    coordinate_t _source_coordinate;

    // buffer for coordinates
    std::vector<coordinate_t> _destination_coordinates;

    // statistics
    std::size_t _base_node_count{0};
    std::size_t _boundary_node_count{0};
    std::size_t _steiner_point_count{0};

    std::size_t _base_node_neighbor_count{0};
    std::size_t _boundary_node_neighbor_count{0};
    std::size_t _steiner_point_neighbor_count{0};

    std::size_t _steiner_point_angle_test_count{0};

    using node_id_type = typename Graph::node_id_type;
    using base_node_id_type = typename Graph::base_topology_type::node_id_type;
    using base_edge_id_type = typename Graph::base_topology_type::edge_id_type;

    [[gnu::hot]]
    [[gnu::always_inline]]
    bool ignore_edge(base_edge_id_type const& edge_id, coordinate_t const& direction) const;

    template<typename NodeCostPair>
    [[gnu::hot]]
    [[gnu::always_inline]]
    void on_edge_neighbors(NodeCostPair const &node, std::vector<NodeCostPair> &out);

    [[gnu::hot]]
    [[gnu::always_inline]]
    node_id_type
    find_min_angle_neighbors_hinted(base_edge_id_type const &edge_id,
                            coordinate_t const &direction, node_id_type const& hint, double& cos, double& cos2);


    [[gnu::hot]]
    [[gnu::always_inline]]
    node_id_type
    find_min_angle_neighbors(base_edge_id_type const &edge_id,
                            coordinate_t const &direction, double& cos, double& cos2);

    template<typename NodeCostPair>
    [[gnu::hot]]
    [[gnu::always_inline]]
    void
    add_min_angle_neighbor(NodeCostPair const &node, base_edge_id_type const &edge_id,
                           double const &max_angle_cos, coordinate_t const &direction, std::vector<NodeCostPair> &out);


    template<typename NodeCostPair>
    void epsilon_spanner(NodeCostPair const &node,
                         base_edge_id_type const &edge_id, double const &max_angle_cos,
                         coordinate_t const &direction,
                         std::vector<NodeCostPair> &out);

    template<typename NodeCostPair>
    void from_boundary_node(NodeCostPair const &node, std::vector<NodeCostPair> &out);

    template<typename NodeCostPair>
    void from_start_node(NodeCostPair const &node, std::vector<NodeCostPair> &out);

    template<typename NodeCostPair>
    void from_base_node(NodeCostPair const &node, std::vector<NodeCostPair> &out);

    template<typename NodeCostPair> requires HasFaceCrossingPredecessor<NodeCostPair, Graph>
    static node_id_type const& find_face_crossing_predecessor(NodeCostPair const &node);

    template<typename NodeCostPair> requires (!HasFaceCrossingPredecessor<NodeCostPair, Graph>)
    node_id_type find_face_crossing_predecessor(NodeCostPair const &node) const;


    template<typename NodeCostPair>
    [[gnu::hot]]
    void from_steiner_node(NodeCostPair const &node, std::vector<NodeCostPair> &out);

public:
    steiner_neighbors(Graph const &graph, Labels &labels)
            : _graph(graph), _labels(labels),
              _spanner_angle{std::clamp(std::numbers::pi * graph.epsilon(), 0.0, std::numbers::pi_v<double>)},
              _spanner_angle_cos{std::cos(_spanner_angle)},
              _max_angle{std::clamp(std::numbers::pi / 2 * graph.epsilon(), std::numeric_limits<double>::min(), std::numbers::pi)},
              _max_angle_cos{std::cos(_max_angle)} { }

    template<typename... Args>
    steiner_neighbors(Graph const &graph, Labels &labels, Args const &...) : steiner_neighbors(graph, labels) { }

    steiner_neighbors(steiner_neighbors &&) noexcept = default;

    steiner_neighbors &operator=(steiner_neighbors &&) noexcept = default;

    template<typename NodeCostPair>
    [[gnu::hot]]
    void operator()(NodeCostPair const &node, std::vector<NodeCostPair> &out);

    std::size_t base_node_count() const { return _base_node_count; };
    std::size_t boundary_node_count() const { return _boundary_node_count; };
    std::size_t steiner_point_count() const { return _steiner_point_count; };

    std::size_t base_node_neighbor_count() const { return _base_node_neighbor_count; };
    std::size_t boundary_node_neighbor_count() const { return _boundary_node_neighbor_count; };
    std::size_t steiner_point_neighbor_count() const { return _steiner_point_neighbor_count; };
    std::size_t steiner_point_angle_test_count() const { return _steiner_point_angle_test_count; };

    [[gnu::hot]]
    double min_angle_relative_value(base_edge_id_type edge_id, coordinate_t const& direction) const {
        coordinate_t const left = _graph.node_coordinates(_graph.base_graph().source(edge_id));
        coordinate_t const src_left  = left - _source_coordinate;
        coordinate_t const right_left = left - _graph.node_coordinates(_graph.base_graph().destination(edge_id));

        double const dist_left = src_left.length();
        double const length = right_left.length();

        // TODO implement it this way to reduce atan calls
        // auto angle0 = std::atan2(src_left.longitude, src_left.latitude);
        // auto angle1 = std::atan2(right_left.longitude, right_left.latitude);
        // auto angle2 = std::atan2(direction.longitude, direction.latitude);

        // double angle_source = std::fabs(angle0 - angle2);
        // angle_source += (angle_source < 0) ? (2 * std::numbers::pi) : 0;
        // double angle_left = std::fabs(angle0 - angle1);
        // angle_left += (angle_left < 0) ? (2 * std::numbers::pi) : 0;

        // angle between src->left and src->intersection
        double const angle_source = inner_angle(src_left, direction);

        // angle between src->left and right->left
        double const angle_left = inner_angle(src_left, right_left);

        // angle between intersection->left and intersection->src
        double const angle_intersection = std::numbers::pi - angle_source - angle_left;

        // compute sin values
        double const sin_source = std::sin(angle_source);
        double const sin_intersection = std::sin(angle_intersection);

        //
        auto const result =  (dist_left / length) * (sin_source / sin_intersection);
        assert(result >= 0. && result < 1.0);
        return result;
    };
};
