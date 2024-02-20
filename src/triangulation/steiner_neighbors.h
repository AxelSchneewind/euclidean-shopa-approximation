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

    coordinate_t::component_type _spanner_angle;
    coordinate_t::component_type _spanner_angle_cos;

    coordinate_t::component_type _max_angle;
    coordinate_t::component_type _max_angle_cos;

    coordinate_t _source_coordinate;

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

    template<typename NodeCostPair>
    [[gnu::hot]]
    [[gnu::always_inline]]
    void on_edge_neighbors(NodeCostPair const &node, std::vector<NodeCostPair> &out, std::vector<coordinate_t> &coordinates_out);


    [[gnu::hot]]
    [[gnu::always_inline]]
    coordinate_t::component_type min_angle_relative_value(base_edge_id_type edge_id, coordinate_t const& direction) const;

    [[gnu::hot]]
    [[gnu::always_inline]]
    coordinate_t::component_type min_angle_relative_value(base_edge_id_type edge_id,
                                                          coordinate_t left,
                                                          coordinate_t right,
                                                          coordinate_t::component_type direction_left,
                                                          coordinate_t::component_type direction_dir) const;

    [[gnu::cold]]
    node_id_type
    find_min_angle_neighbors_hinted(base_edge_id_type const &edge_id,
                            coordinate_t const &direction, node_id_type const& hint, coordinate_t::component_type & cos, coordinate_t::component_type& cos2);


    [[gnu::cold]]
    node_id_type
    find_min_angle_neighbors(base_edge_id_type const &edge_id,
                            coordinate_t const &direction, coordinate_t::component_type & cos, coordinate_t::component_type & cos2);

    template<typename NodeCostPair>
    [[gnu::hot]]
    [[gnu::always_inline]]
    void
    add_min_angle_neighbor( NodeCostPair const &node, base_edge_id_type const &edge_id,
            coordinate_t const& left, coordinate_t const& right, coordinate_t::component_type angle_left_dir, coordinate_t::component_type angle_dir,
            std::vector<NodeCostPair> &out, std::vector<coordinate_t> &out_coordinates);


    template<typename NodeCostPair>
    [[gnu::hot]]
    [[gnu::always_inline]]
    void
    add_min_angle_neighbor(NodeCostPair const &node, base_edge_id_type const &edge_id,
                           coordinate_t const&direction, std::vector<NodeCostPair> &out, std::vector<coordinate_t> &out_coordinates);


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
              _max_angle{std::clamp(std::numbers::pi / 2 * graph.epsilon(), std::numeric_limits<coordinate_t::component_type>::min(), std::numbers::pi / 4)},
              _max_angle_cos{std::cos(_max_angle)} { }

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
