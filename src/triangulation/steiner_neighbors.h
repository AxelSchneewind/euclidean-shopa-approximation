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
    { n.face_crossing_predecessor() };
} && requires( NodeCostPair const &n) {
    { n.face_crossing_predecessor() };
};



enum class Configuration {
    PARAM,
    ATAN2,
    BINSEARCH,
    LINEAR      // linear search, currently not implemented
};

template<typename Graph, typename Labels, Configuration Config = Configuration::PARAM>
struct steiner_neighbors {
private:
    using node_id_type = typename Graph::node_id_type;
    using base_node_id_type = typename Graph::base_topology_type::node_id_type;
    using base_edge_id_type = typename Graph::base_topology_type::edge_id_type;

    std::shared_ptr<Graph> _graph;
    std::shared_ptr<Labels> _labels;

    // angle of a subcone of an epsilon-spanner
    coordinate_t::component_type _spanner_angle;
    coordinate_t::component_type _spanner_angle_cos;
    coordinate_t::component_type _spanner_angle_sin;

    // maximal angle between two adjacent steiner points, seen from a reachable edge
    coordinate_t::component_type _max_angle;

    // for storing information on the current neighbor query
    node_id_type _source;
    coordinate_t _source_coordinate;
    coordinate_t _direction;

    bool _first_call{true};

    // statistics
    std::size_t _base_node_count{0};
    std::size_t _boundary_node_count{0};
    std::size_t _steiner_point_count{0};

    std::size_t _base_node_neighbor_count{0};
    std::size_t _boundary_node_neighbor_count{0};
    std::size_t _steiner_point_neighbor_count{0};

    std::size_t _steiner_point_angle_test_count{0};     // for linear/binary search: number of iterations/tests

    void init(node_id_type /*source*/, node_id_type /*target*/) {
        _first_call = true;
    }

    template<typename NodeCostPair>
    [[using gnu : hot]]
    void insert(node_id_type const& neighbor, coordinate_t const& neighbor_coordinate, NodeCostPair const& current, std::vector<NodeCostPair> &out, std::vector<coordinate_t> &coordinates_out) const;

    template<typename NodeCostPair>
    [[using gnu : hot]]
    void insert(node_id_type const& neighbor, NodeCostPair const& current, std::vector<NodeCostPair> &out, std::vector<coordinate_t> &coordinates_out) const;

    template<typename NodeCostPair>
    [[gnu::hot]]
    void on_edge_neighbors(NodeCostPair const &node, std::vector<NodeCostPair> &out, std::vector<coordinate_t> &coordinates_out);

    [[gnu::hot]]
    coordinate_t::component_type min_angle_relative_value_matmul(base_edge_id_type edge_id, coordinate_t direction) const requires (Configuration::PARAM == Config);

    [[gnu::hot]]
    coordinate_t::component_type min_angle_relative_value_atan2(base_edge_id_type edge_id, coordinate_t const& direction) const requires (Configuration::ATAN2 == Config);

    [[gnu::hot]]
    coordinate_t::component_type min_angle_relative_value_atan2(coordinate_t left,
                                                                coordinate_t right,
                                                                coordinate_t::component_type direction_left,
                                                                coordinate_t::component_type direction_dir) const requires (Configuration::ATAN2 == Config);

    [[gnu::hot]]
    node_id_type
    min_angle_neighbor_matmul(base_edge_id_type const &edge_id,
                              coordinate_t const &direction) requires (Configuration::PARAM == Config);

    [[gnu::hot]]
    node_id_type min_angle_neighbor_atan2(base_edge_id_type edge_id, const coordinate_t &direction) const requires (Configuration::ATAN2 == Config);


    [[gnu::hot]]
    node_id_type
    min_angle_neighbor_binary_search(base_edge_id_type const &edge_id,
                                     const coordinate_t &direction) requires (Configuration::BINSEARCH == Config);

    template<typename NodeCostPair>
    [[gnu::hot]]
    void
    add_min_angle_neighbor(NodeCostPair const &node, coordinate_t const& direction,
            std::vector<NodeCostPair> &out, std::vector<coordinate_t> &out_coordinates);

    template<typename NodeCostPair>
    void epsilon_spanner(NodeCostPair const &node,
                         base_edge_id_type const &edge_id, coordinate_t::component_type const &max_angle_cos,
                         std::vector<NodeCostPair> &out, std::vector<coordinate_t> &out_coordinates);

    template<typename NodeCostPair>
    void from_boundary_node(NodeCostPair const &node, std::vector<NodeCostPair> &out, std::vector<coordinate_t> &out_coordinates);

    template<typename NodeCostPair>
    void from_start_node(NodeCostPair const &node, std::vector<NodeCostPair> &out, std::vector<coordinate_t> &out_coordinates);

    template<typename NodeCostPair>
    void from_base_node(NodeCostPair const &node, std::vector<NodeCostPair> &out, std::vector<coordinate_t> &out_coordinates);

    template<typename NodeCostPair> requires HasFaceCrossingPredecessor<NodeCostPair, Graph>
    static node_id_type const& find_face_crossing_predecessor(NodeCostPair const &node);

    template<typename NodeCostPair> requires (!HasFaceCrossingPredecessor<NodeCostPair, Graph> && !HasFaceCrossingPredecessor<typename Labels::value_type, Graph>)
    node_id_type find_face_crossing_predecessor(NodeCostPair const &node) const;
    template<typename NodeCostPair> requires (!HasFaceCrossingPredecessor<NodeCostPair, Graph> && HasFaceCrossingPredecessor<typename Labels::value_type, Graph>)
    node_id_type const& find_face_crossing_predecessor(NodeCostPair const &node) const;

    template<typename NodeCostPair>
    [[gnu::hot]]
    void from_steiner_node(NodeCostPair const &node, std::vector<NodeCostPair> &out, std::vector<coordinate_t> &out_coordinates);

public:
    steiner_neighbors(std::shared_ptr<Graph> graph, std::shared_ptr<Labels> labels)
            : _graph(std::move(graph))
            , _labels(std::move(labels))
            , _spanner_angle{std::clamp(std::numbers::pi * _graph->epsilon() / 2, 0.0, std::numbers::pi_v<coordinate_t::component_type> / 2)}
            , _spanner_angle_cos{std::cos(_spanner_angle)}
            , _spanner_angle_sin{std::sin(_spanner_angle)}
            , _max_angle{std::clamp(std::numbers::pi / 2 * _graph->epsilon(), std::numeric_limits<coordinate_t::component_type>::min(), std::numbers::pi / 4)}
            {}

    template<typename... Args>
    steiner_neighbors(std::shared_ptr<Graph> graph, Labels &labels, Args const &...) : steiner_neighbors(graph, labels) { }

    steiner_neighbors(steiner_neighbors const&) noexcept = default;
    steiner_neighbors &operator=(steiner_neighbors const&) noexcept = default;

    steiner_neighbors(steiner_neighbors &&) noexcept = default;
    steiner_neighbors &operator=(steiner_neighbors &&) noexcept = default;

    template<typename NodeCostPair>
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
