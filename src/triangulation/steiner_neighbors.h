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
} && requires(NodeCostPair &n) {
    { n.face_crossing_predecessor() };
} && requires(NodeCostPair const &n) {
    { n.face_crossing_predecessor() };
};

enum class Pruning : int {
    UNPRUNED,           // search full graph, currently not implemented
    PRUNE_DEFAULT,      // default pruning
    MinBendingAngleESpanner // alternative pruning, minimizing bending angles
};

enum class NeighborFindingAlgorithm : int {
    PARAM,
    ATAN2,
    BINSEARCH,
    LINEAR      // linear search, currently not implemented
};

// TODO extract some behaviour into subclasses
template<typename Graph, typename Labels, Pruning Simplifications = Pruning::PRUNE_DEFAULT, NeighborFindingAlgorithm Config = NeighborFindingAlgorithm::PARAM>
struct steiner_neighbors {
private:
    using node_id_type = typename Graph::node_id_type;
    using base_node_id_type = typename Graph::base_topology_type::node_id_type;
    using base_edge_id_type = typename Graph::base_topology_type::edge_id_type;

    // if set to true, the implementation does not search for the closest neighbor in a sub-cone but the one with lowest bending angle
    static constexpr bool simplify_epsilon_spanner = (Simplifications == Pruning::MinBendingAngleESpanner);

    //
    std::shared_ptr<Graph> _graph;
    std::shared_ptr<Labels> _labels;

    // angle of a sub-cone of an epsilon-spanner
    coordinate_t::component_type _spanner_angle;
    coordinate_t::component_type _spanner_angle_sin;


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

    std::size_t _steiner_point_angle_test_count{0};     // number of segments checked

    // getting the face crossing predecessor depending on whether it is stored in queue or labels or implicitly in tree
    template<typename NodeCostPair>
    requires HasFaceCrossingPredecessor<NodeCostPair, Graph>
    static node_id_type const &find_face_crossing_predecessor(NodeCostPair const &node);

    template<typename NodeCostPair>
    requires (!HasFaceCrossingPredecessor<NodeCostPair, Graph> &&
              !HasFaceCrossingPredecessor<typename Labels::value_type, Graph>)
    node_id_type find_face_crossing_predecessor(NodeCostPair const &node) const;

    template<typename NodeCostPair>
    requires (!HasFaceCrossingPredecessor<NodeCostPair, Graph> &&
              HasFaceCrossingPredecessor<typename Labels::value_type, Graph>)
    node_id_type const &find_face_crossing_predecessor(NodeCostPair const &node) const;


    // inserting a neighbor into the output buffer
    template<typename NodeCostPair>
    [[using gnu : hot]]
    void insert(node_id_type const &neighbor, coordinate_t const &neighbor_coordinate, NodeCostPair const &current,
                std::vector<NodeCostPair> &out, std::vector<coordinate_t> &coordinates_out) const;

    template<typename NodeCostPair>
    [[using gnu : hot]]
    void insert(node_id_type const &neighbor, NodeCostPair const &current, std::vector<NodeCostPair> &out,
                std::vector<coordinate_t> &coordinates_out) const;

    // edge using segments
    template<typename NodeCostPair>
    [[gnu::hot]]
    void on_edge_neighbors(NodeCostPair const &node, std::vector<NodeCostPair> &out,
                           std::vector<coordinate_t> &coordinates_out);

    template<typename NodeCostPair>
    [[gnu::hot]]
    void vertex_neighbors(NodeCostPair const &node, std::vector<NodeCostPair> &out,
                          std::vector<coordinate_t> &coordinates_out);

    // finding the intersection of a ray with an edge, depending on algorithm selected
    [[using gnu : hot]]
    coordinate_t::component_type
    min_angle_relative_value_matmul(base_edge_id_type edge_id, coordinate_t direction) const requires (
    NeighborFindingAlgorithm::PARAM == Config);

    [[using gnu : hot]]
    coordinate_t::component_type
    min_angle_relative_value_atan2(base_edge_id_type edge_id, coordinate_t const &direction) const requires (
    NeighborFindingAlgorithm::ATAN2 == Config);

    [[using gnu : hot]]
    coordinate_t::component_type min_angle_relative_value_atan2(coordinate_t left,
                                                                coordinate_t right,
                                                                coordinate_t::component_type direction_left,
                                                                coordinate_t::component_type direction_dir) const requires (
    NeighborFindingAlgorithm::ATAN2 == Config);

    [[using gnu : hot]]
    node_id_type
    min_angle_neighbor_matmul(base_edge_id_type const &edge_id,
                              coordinate_t const &direction) requires (NeighborFindingAlgorithm::PARAM == Config);

    [[using gnu : hot]]
    node_id_type min_angle_neighbor_atan2(base_edge_id_type edge_id, const coordinate_t &direction) const
    requires (NeighborFindingAlgorithm::ATAN2 == Config);


    [[using gnu : hot]]
    node_id_type
    min_angle_neighbor_binary_search(base_edge_id_type const &edge_id,
                                     const coordinate_t &direction) requires (
    NeighborFindingAlgorithm::BINSEARCH == Config ||
    NeighborFindingAlgorithm::LINEAR == Config);

    /**
     * adds the sub-cones to the left and right of the ray from current node in the given direction
     * @tparam NodeCostPair
     * @param node
     * @param direction
     * @param out
     * @param out_coordinates
     */
    template<typename NodeCostPair>
    [[gnu::hot]]
    void
    add_min_angle_neighbor(NodeCostPair const &node, coordinate_t const &direction,
                           std::vector<NodeCostPair> &out, std::vector<coordinate_t> &out_coordinates) requires (Pruning::UNPRUNED != Simplifications);

    /**
     * adds all segments to points on the edge intersected by the ray from current node in the given direction
     * @tparam NodeCostPair
     * @param node
     * @param direction
     * @param out
     * @param out_coordinates
     */
    template<typename NodeCostPair>
    [[gnu::hot]]
    void
    add_min_angle_neighbor(NodeCostPair const &node, coordinate_t const &direction,
                           std::vector<NodeCostPair> &out, std::vector<coordinate_t> &out_coordinates) requires (Pruning::UNPRUNED == Simplifications);


    // epsilon spanners depending on pruning
    // TODO document, add overload for epsilon spanner with angle given
    /**
     * generates an epsilon like described in the thesis
     * @tparam NodeCostPair
     * @param node
     * @param edge_id
     * @param out
     * @param out_coordinates
     */
    template<typename NodeCostPair>
    void epsilon_spanner(NodeCostPair const &node,
                         base_edge_id_type const &edge_id,
                         std::vector<NodeCostPair> &out, std::vector<coordinate_t> &out_coordinates) requires (Pruning::UNPRUNED != Simplifications);

    /**
     * generates all outgoing edges
     * @tparam NodeCostPair
     * @param node
     * @param edge_id
     * @param out
     * @param out_coordinates
     */
    template<typename NodeCostPair>
    void epsilon_spanner(NodeCostPair const &node,
                         base_edge_id_type const &edge_id,
                         std::vector<NodeCostPair> &out, std::vector<coordinate_t> &out_coordinates) requires (Pruning::UNPRUNED == Simplifications);


    // handling the different cases
    template<typename NodeCostPair>
    void from_boundary_node(NodeCostPair const &node, std::vector<NodeCostPair> &out,
                            std::vector<coordinate_t> &out_coordinates);

    template<typename NodeCostPair>
    void from_start_node(NodeCostPair const &node, std::vector<NodeCostPair> &out,
                         std::vector<coordinate_t> &out_coordinates);

    template<typename NodeCostPair>
    void from_base_node(NodeCostPair const &node, std::vector<NodeCostPair> &out,
                        std::vector<coordinate_t> &out_coordinates);

    template<typename NodeCostPair>
    [[gnu::hot]]
    void from_steiner_node(NodeCostPair const &node, std::vector<NodeCostPair> &out,
                           std::vector<coordinate_t> &out_coordinates);

public:
    steiner_neighbors(std::shared_ptr<Graph> graph, std::shared_ptr<Labels> labels)
            : _graph(std::move(graph)), _labels(std::move(labels)), _spanner_angle{
            std::clamp(std::numbers::pi * _graph->epsilon() / 2, 0.0,
                       std::numbers::pi_v<coordinate_t::component_type> / 4)},
              _spanner_angle_sin{std::sin(_spanner_angle)} {}

    template<typename... Args>
    steiner_neighbors(std::shared_ptr<Graph> graph, Labels &labels, Args const &...) : steiner_neighbors{graph,
                                                                                                         labels} {}

    steiner_neighbors(steiner_neighbors const &) noexcept = default;

    steiner_neighbors &operator=(steiner_neighbors const &) noexcept = default;

    steiner_neighbors(steiner_neighbors &&) noexcept = default;

    steiner_neighbors &operator=(steiner_neighbors &&) noexcept = default;

    [[gnu::cold]]
    void init(node_id_type /*source*/, node_id_type /*target*/) {
	// reset statistics
    	_base_node_count = 0;
    	_boundary_node_count = 0;
    	_steiner_point_count = 0;

    	_base_node_neighbor_count = 0;
    	_boundary_node_neighbor_count = 0;
    	_steiner_point_neighbor_count = 0;

    	_steiner_point_angle_test_count = 0;
    }


    // entry points
    template<typename NodeCostPair>
    void operator()(NodeCostPair const &node, std::vector<NodeCostPair> &out);

    template<typename NodeCostPair>
    void
    operator()(NodeCostPair const &node, std::vector<NodeCostPair> &out, std::vector<coordinate_t> &coordinates_out);


    // getters for statistics
    std::size_t base_node_count() const { return _base_node_count; };

    std::size_t boundary_node_count() const { return _boundary_node_count; };

    std::size_t steiner_point_count() const { return _steiner_point_count; };

    std::size_t base_node_neighbor_count() const { return _base_node_neighbor_count; };

    std::size_t boundary_node_neighbor_count() const { return _boundary_node_neighbor_count; };

    std::size_t steiner_point_neighbor_count() const { return _steiner_point_neighbor_count; };

    std::size_t steiner_point_angle_test_count() const { return _steiner_point_angle_test_count; };
};
