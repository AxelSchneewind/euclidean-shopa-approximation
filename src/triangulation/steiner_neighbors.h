#pragma once

#include "../graph/base_types.h"

#include <concepts>
#include <cstddef>

#include <numbers>
#include <cmath>
#include <vector>


template<typename NodeCostPair, typename Graph>
concept HasFaceCrossingPredecessor = requires {
    typename Graph::node_id_type;
} && requires( NodeCostPair &n) {
    { n.face_crossing_predecessor() } -> std::convertible_to<typename Graph::node_id_type&>;
} && requires( NodeCostPair const &n) {
    { n.face_crossing_predecessor() } -> std::convertible_to<typename Graph::node_id_type>;
};



enum class Configuration {
    LINALG,
    BINSEARCH,
    ATAN2
};

template<typename Graph, typename Labels, Configuration Config = Configuration::LINALG>
struct steiner_neighbors {
private:
    using node_id_type = typename Graph::node_id_type;
    using base_node_id_type = typename Graph::base_topology_type::node_id_type;
    using base_edge_id_type = typename Graph::base_topology_type::edge_id_type;

    Graph const &_graph;
    Labels &_labels;

    coordinate_t::component_type _spanner_angle;
    coordinate_t::component_type _spanner_angle_cos;

    coordinate_t::component_type _max_angle;

    node_id_type _source;
    coordinate_t _source_coordinate;

    // statistics
    std::size_t _base_node_count{0};
    std::size_t _boundary_node_count{0};
    std::size_t _steiner_point_count{0};

    std::size_t _base_node_neighbor_count{0};
    std::size_t _boundary_node_neighbor_count{0};
    std::size_t _steiner_point_neighbor_count{0};

    std::size_t _steiner_point_angle_test_count{0};

    template<typename NodeCostPair>
    [[gnu::hot]]
    [[gnu::always_inline]]
    void on_edge_neighbors(NodeCostPair const &node, std::vector<NodeCostPair> &out, std::vector<coordinate_t> &coordinates_out);

    [[gnu::hot]]
    [[gnu::always_inline]]
    coordinate_t::component_type min_angle_relative_value_matmul(base_edge_id_type edge_id, coordinate_t direction) const;

    [[gnu::hot]]
    [[gnu::always_inline]]
    coordinate_t::component_type min_angle_relative_value_atan2(base_edge_id_type edge_id, coordinate_t const& direction) const;

    [[gnu::hot]]
    [[gnu::always_inline]]
    coordinate_t::component_type min_angle_relative_value_atan2(base_edge_id_type edge_id,
                                                                coordinate_t left,
                                                                coordinate_t right,
                                                                coordinate_t::component_type direction_left,
                                                                coordinate_t::component_type direction_dir) const;

    [[gnu::hot]]
    [[gnu::always_inline]]
    node_id_type
    min_angle_neighbor_matmul(base_edge_id_type const &edge_id,
                              coordinate_t const &direction);

    [[gnu::hot]]
    [[gnu::always_inline]]
    node_id_type min_angle_neighbor_atan2(base_edge_id_type edge_id, const coordinate_t &direction) const;


    [[gnu::hot]]
    [[gnu::always_inline]]
    node_id_type
    min_angle_neighbor_binary_search(base_edge_id_type const &edge_id,
                                     const coordinate_t &direction);

    template<typename NodeCostPair>
    [[gnu::hot]]
    [[gnu::always_inline]]
    void
    add_min_angle_neighbor(NodeCostPair const &node, coordinate_t const& direction,
            std::vector<NodeCostPair> &out, std::vector<coordinate_t> &out_coordinates);

    template<typename NodeCostPair>
    void epsilon_spanner(NodeCostPair const &node,
                         base_edge_id_type const &edge_id, coordinate_t::component_type const &max_angle_cos,
                         coordinate_t const &direction,
                         std::vector<NodeCostPair> &out, std::vector<coordinate_t> &out_coordinates);

    template<typename NodeCostPair>
    void from_boundary_node(NodeCostPair const &node, std::vector<NodeCostPair> &out, std::vector<coordinate_t> &out_coordinates);

    template<typename NodeCostPair>
    void from_start_node(NodeCostPair const &node, std::vector<NodeCostPair> &out, std::vector<coordinate_t> &out_coordinates);

    template<typename NodeCostPair>
    void from_base_node(NodeCostPair const &node, std::vector<NodeCostPair> &out, std::vector<coordinate_t> &out_coordinates);

    template<typename NodeCostPair> requires HasFaceCrossingPredecessor<NodeCostPair, Graph>
    static node_id_type const& find_face_crossing_predecessor(NodeCostPair const &node);

    template<typename NodeCostPair> requires (!HasFaceCrossingPredecessor<NodeCostPair, Graph>)
    node_id_type find_face_crossing_predecessor(NodeCostPair const &node) const;


    template<typename NodeCostPair>
    [[gnu::hot]]
    void from_steiner_node(NodeCostPair const &node, std::vector<NodeCostPair> &out, std::vector<coordinate_t> &out_coordinates);

public:
    steiner_neighbors(Graph const &graph, Labels &labels)
            : _graph(graph), _labels(labels),
              _spanner_angle{std::clamp(std::numbers::pi * graph.epsilon(), 0.0, std::numbers::pi_v<coordinate_t::component_type>)},
              _spanner_angle_cos{std::cos(_spanner_angle)},
              _max_angle{std::clamp(std::numbers::pi / 2 * graph.epsilon(), std::numeric_limits<coordinate_t::component_type>::min(), std::numbers::pi / 4)}
              {}

    template<typename... Args>
    steiner_neighbors(Graph const &graph, Labels &labels, Args const &...) : steiner_neighbors(graph, labels) { }

    steiner_neighbors(steiner_neighbors &&) noexcept = default;

    steiner_neighbors &operator=(steiner_neighbors &&) noexcept = default;

    template<typename NodeCostPair>
    [[gnu::hot]]
    void operator()(NodeCostPair const &node, std::vector<NodeCostPair> &out);

    template<typename NodeCostPair>
    [[gnu::hot]]
    void operator()(NodeCostPair const &node, std::vector<NodeCostPair> &out, std::vector<coordinate_t>& coordinates_out);

    std::size_t base_node_count() const { return _base_node_count; };
    std::size_t boundary_node_count() const { return _boundary_node_count; };
    std::size_t steiner_point_count() const { return _steiner_point_count; };

    std::size_t base_node_neighbor_count() const { return _base_node_neighbor_count; };
    std::size_t boundary_node_neighbor_count() const { return _boundary_node_neighbor_count; };
    std::size_t steiner_point_neighbor_count() const { return _steiner_point_neighbor_count; };
    std::size_t steiner_point_angle_test_count() const { return _steiner_point_angle_test_count; };
};
