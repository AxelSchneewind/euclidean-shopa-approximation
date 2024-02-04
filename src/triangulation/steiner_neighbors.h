#pragma once

#include "../graph/base_types.h"

#include <cmath>
#include <concepts>
#include <cstddef>
#include <math.h>
#include <vector>

template<typename NodeCostPair, typename Graph>
concept HasFaceCrossingPredecessor = requires { typename Graph::node_id_type; }
                                    && requires(
        NodeCostPair &n) {{ n.face_crossing_predecessor() } -> std::convertible_to<typename Graph::node_id_type &>; }
                                    && requires(
        NodeCostPair const &n) {{ n.face_crossing_predecessor() } -> std::convertible_to<typename Graph::node_id_type>; };

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
    Labels const &_labels;

    double _spanner_angle;
    double _spanner_angle_cos;

    double _max_angle;
    double _max_angle_cos;

    coordinate_t _source_coordinate;

    // buffer for coordinates
    std::vector<coordinate_t> _destination_coordinates;

    std::size_t _base_node_count{0};
    std::size_t _boundary_node_count{0};
    std::size_t _steiner_point_count{0};

    std::size_t _base_node_neighbor_count{0};
    std::size_t _boundary_node_neighbor_count{0};
    std::size_t _steiner_point_neighbor_count{0};

    using node_id_type = typename Graph::node_id_type;
    using base_node_id_type = typename Graph::base_topology_type::node_id_type;
    using base_edge_id_type = typename Graph::base_topology_type::edge_id_type;

    [[gnu::hot]]
    bool ignore_edge(base_edge_id_type const& edge_id, coordinate_t const& direction);

    template<typename NodeCostPair>
    [[gnu::always_inline]]
    void on_edge_neighbors(NodeCostPair const &node, std::vector<NodeCostPair> &out);


    template<typename NodeCostPair>
    [[gnu::hot]]
    [[gnu::always_inline]]
    node_id_type
    find_min_angle_neighbor(base_edge_id_type const &edge_id,
                            coordinate_t const &direction, double& cos);

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
    node_id_type find_face_crossing_predecessor(NodeCostPair const &node);


    template<typename NodeCostPair>
    [[gnu::hot]]
    void from_steiner_node(NodeCostPair const &node, std::vector<NodeCostPair> &out);

public:
    steiner_neighbors(Graph const &graph, Labels const &labels)
            : _graph(graph), _labels(labels),
              _spanner_angle{std::clamp(M_PI_2 * graph.epsilon(), 0.0, M_PI)},
              _spanner_angle_cos{std::cos(_spanner_angle)},
              _max_angle{std::clamp(M_PI_2 * graph.epsilon() * 1.1, _spanner_angle, M_PI)},
              _max_angle_cos{std::cos(_max_angle)} { }

    template<typename... Args>
    steiner_neighbors(Graph const &graph, Labels const &labels, Args const &...) : steiner_neighbors(graph, labels) { }

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
};
