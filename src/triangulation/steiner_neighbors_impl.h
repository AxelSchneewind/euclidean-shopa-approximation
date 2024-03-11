#include "steiner_neighbors.h"
#include "../graph/base_types.h"
#include "../graph/geometry.h"

#include <array>
#include <cmath>
#include <numbers>
#include <vector>
#include <algorithm>


template<typename Graph, typename Labels, Configuration Config>
template<typename NodeCostPair>
void steiner_neighbors<Graph, Labels, Config>::insert(node_id_type neighbor, NodeCostPair current,
                                                      std::vector<NodeCostPair> &out,
                                                      std::vector<coordinate_t> &out_coordinates) const {
    coordinate_t destination_coordinate{_graph->node_coordinates(neighbor)};
    NodeCostPair new_ncp{};
    new_ncp.node() = neighbor;
    new_ncp.distance() = current.distance();
    if constexpr(HasHeuristic<NodeCostPair>)
        new_ncp.heuristic() = current.distance();
    out.emplace_back(new_ncp);
    out_coordinates.emplace_back(destination_coordinate);
}




template<typename Graph, typename Labels, Configuration Config>
template<typename NodeCostPair>
requires HasFaceCrossingPredecessor<NodeCostPair, Graph>
typename Graph::node_id_type const &
steiner_neighbors<Graph, Labels, Config>::find_face_crossing_predecessor(const NodeCostPair &node) {
    assert(!optional::is_none(node.face_crossing_predecessor()));
    return node.face_crossing_predecessor();
}

template<typename Graph, typename Labels, Configuration Config>
template<typename NodeCostPair>
requires (!HasFaceCrossingPredecessor<NodeCostPair, Graph> && HasFaceCrossingPredecessor<typename Labels::value_type, Graph>)
typename Graph::node_id_type const&
steiner_neighbors<Graph, Labels, Config>::find_face_crossing_predecessor(const NodeCostPair &node) const {
    return _labels->at(node.node()).face_crossing_predecessor();
}

template<typename Graph, typename Labels, Configuration Config>
template<typename NodeCostPair>
requires (!HasFaceCrossingPredecessor<NodeCostPair, Graph> && !HasFaceCrossingPredecessor<typename Labels::value_type, Graph>)
typename Graph::node_id_type
steiner_neighbors<Graph, Labels, Config>::find_face_crossing_predecessor(const NodeCostPair &node) const {
    auto const &node_id = node.node();
    auto const &reached_from = node.predecessor();

    // find first predecessor on different edge
    auto closer_face_crossing_predecessor = node_id;
    auto face_crossing_predecessor = reached_from;
    while ((face_crossing_predecessor.edge == node_id.edge ||
            _graph->is_base_node(closer_face_crossing_predecessor))
           && face_crossing_predecessor != closer_face_crossing_predecessor
            ) [[likely]] {
        closer_face_crossing_predecessor = face_crossing_predecessor;
        face_crossing_predecessor = _labels->at(face_crossing_predecessor).predecessor();
    }
    return face_crossing_predecessor;
}

template<typename Graph, typename Labels, Configuration Config>
template<typename NodeCostPair>
void steiner_neighbors<Graph, Labels, Config>::operator()(const NodeCostPair &node, std::vector<NodeCostPair> &out,
                                                          std::vector<coordinate_t> &coordinates_out) {
    auto const &node_id = node.node();
    assert(!optional::is_none(node_id));

    _source = node_id;
    _source_coordinate = _graph->node_coordinates(node_id);

    //
    bool const is_start_node = _first_call;
    _first_call = false;
    bool const is_base_node = _graph->is_base_node(node_id);
    bool is_boundary_node = false;

    if constexpr (HasFaceCrossingPredecessor<NodeCostPair, Graph>) {
        assert(is_start_node ||
               (!optional::is_none(node.face_crossing_predecessor()) &&
                node.face_crossing_predecessor() != node.node()));
    }

    if (!is_base_node && !is_start_node) {
        from_steiner_node(node, out, coordinates_out);
    } else if (!is_base_node) {
        from_start_node(node, out, coordinates_out);
    } else {
        auto&& base_node_id = _graph->base_node_id(node_id);
        is_boundary_node = _graph->is_boundary_node(base_node_id);
        if (is_boundary_node) {
            from_base_node(node, out, coordinates_out);
        } else {
            from_boundary_node(node, out, coordinates_out);
        }
    }

    // set face crossing predecessor of neighbors
    if constexpr (HasFaceCrossingPredecessor<typename Labels::value_type, Graph>) {
        // if this node is a boundary node, set it as the new face crossing predecessor
        auto& label = (*_labels)[node.node()];
        auto fcp = (optional::is_none(label.face_crossing_predecessor()) || is_boundary_node)
                   ? node.node() : label.face_crossing_predecessor();

        assert(!optional::is_none(fcp));
        for (auto& ncp : out) [[likely]] {
            (*_labels)[ncp.node()].face_crossing_predecessor() = fcp;
        }
    } else if constexpr (HasFaceCrossingPredecessor<NodeCostPair, Graph>) {
        auto fcp = (optional::is_none(node.face_crossing_predecessor()) || is_boundary_node)
                   ? node.node() : node.face_crossing_predecessor();

        assert(!optional::is_none(fcp));
        for (auto& ncp : out) [[likely]] {
            ncp.face_crossing_predecessor() = fcp;
        }
    }

    // compute distances (can be vectorized)
    if constexpr(HasDistance<NodeCostPair>) {
        assert(out.size() == coordinates_out.size());
        for (size_t e = 0; e < out.size(); ++e) [[likely]] {
            out[e].distance() = _labels->at(node.node()).distance() + distance(_source_coordinate, coordinates_out[e]);
            assert(out[e].distance() > 0);
        }
    }
}

template<typename Graph, typename Labels, Configuration Config>
template<typename NodeCostPair>
void steiner_neighbors<Graph, Labels, Config>::operator()(const NodeCostPair &node, std::vector<NodeCostPair> &out) {
    static std::vector<coordinate_t> coordinates;
    operator()(node, out, coordinates);
    coordinates.clear();
}


// RELATIVE VALUES
template<typename Graph, typename Labels, Configuration Config>
coordinate_t::component_type
steiner_neighbors<Graph, Labels, Config>::min_angle_relative_value_matmul(base_edge_id_type edge_id,
                                                                          coordinate_t direction_source) const {
    // compute intersection point_source + b*direction_source = point_target + result * direction_target

    // point vectors
    // coordinate_t     point_source = _source_coordinate;
    coordinate_t        point_target = _graph->node_coordinates_first(edge_id);

    // direction vectors
    //coordinate_t      direction_source = direction;
    coordinate_t        direction_target = _graph->node_coordinates_last(edge_id) - point_target;

    // difference between point vectors
    // coordinate_t const  right_side = point_target - point_source;

    // system matrix
    // std::array<coordinate_t::component_type, 4> system {
    //     -direction_target.longitude, direction_source.longitude,
    //     -direction_target.latitude,  direction_source.latitude
    // };

    // inverse matrix
    // std::array<coordinate_t::component_type, 4> inverse {
    //      direction_source.latitude, -direction_source.longitude,
    //      direction_target.latitude, -direction_target.longitude
    // };

    //
    coordinate_t const right_side = point_target - _source_coordinate;
    coordinate_t::component_type result = (right_side.longitude * direction_source.latitude) - (right_side.latitude * direction_source.longitude);
    double factor = (-direction_target.longitude * direction_source.latitude) + (direction_source.longitude * direction_target.latitude);
    return result / factor;
}

template<typename Graph, typename Labels, Configuration Config>
coordinate_t::component_type
steiner_neighbors<Graph, Labels, Config>::min_angle_relative_value_atan2(base_edge_id_type edge_id,
                                                                         coordinate_t const &direction) const {
    // src->left, right-left
    coordinate_t source_left;
    coordinate_t right_left;
    {
        coordinate_t const src = _graph->node_coordinates_first(edge_id);
        source_left = src - _source_coordinate;
        right_left = src - _graph->node_coordinates_last(edge_id);
    }

    coordinate_t::component_type angle_source;
    coordinate_t::component_type angle_left;
    {
        coordinate_t::component_type angles[3];
        angles[0] = std::atan2(source_left);
        angles[1] = std::atan2(right_left);
        angles[2] = std::atan2(direction);

        // angle between src->left and src->intersection
        angle_source = std::fabs(angles[0] - angles[2]); // inner_angle(coordinates[1], direction);

        // angle between src->left and right->left
        angle_left = std::fabs(angles[0] - angles[1]); // inner_angle(coordinates[1], coordinates[0]);
    }

    //
    angle_source = (angle_source > std::numbers::pi) ? (2 * std::numbers::pi) - angle_source : angle_source;
    angle_left = (angle_left > std::numbers::pi) ? (2 * std::numbers::pi) - angle_left : angle_left;

    coordinate_t::component_type result;
    {
        // side lengths
        coordinate_t::component_type const dist_left = source_left.sqr_length();
        coordinate_t::component_type const length = right_left.sqr_length();

        // compute sin values
        coordinate_t::component_type const sin_source = std::sin(angle_source);

        // angle between intersection->left and intersection->src
        // std::sin(std::numbers::pi - angle_source - angle_left) is equal to:
        coordinate_t::component_type const sin_intersection = std::sin(angle_source + angle_left);

        // found using law of sines
        result = std::sqrt(dist_left / length) * (sin_source / sin_intersection);
        assert(result >= -0.0001 && result <= 1.0001);
    }

    return std::clamp(result, 0.0, 1.0);
}

template<typename Graph, typename Labels, Configuration Config>
coordinate_t::component_type
steiner_neighbors<Graph, Labels, Config>::min_angle_relative_value_atan2(coordinate_t left,
                                                                         coordinate_t right,
                                                                         coordinate_t::component_type direction_left,
                                                                         coordinate_t::component_type direction_dir) const {
    // left->right
    right = left - right;
    // source->left
    left -= _source_coordinate;

    coordinate_t::component_type angle_source;
    coordinate_t::component_type angle_left;
    {
        // angle between src->left and src->intersection
        angle_source = std::fabs(direction_left - direction_dir);

        // angle between src->left and right->left
        angle_left = std::fabs(direction_left - std::atan2(right));

        //
        angle_source = (angle_source > std::numbers::pi) ? (2 * std::numbers::pi) - angle_source : angle_source;
        angle_left = (angle_left > std::numbers::pi) ? (2 * std::numbers::pi) - angle_left : angle_left;
    }

    coordinate_t::component_type result;
    {
        // side lengths
        coordinate_t::component_type const dist_left = left.sqr_length();
        coordinate_t::component_type const length = right.sqr_length();

        // compute sin values
        coordinate_t::component_type const sin_source = std::sin(angle_source);

        // angle between intersection->left and intersection->src
        // std::sin(std::numbers::pi - angle_source - angle_left) is equal to:
        coordinate_t::component_type const sin_intersection = std::sin(angle_source + angle_left);

        // found using law of sines
        result = std::sqrt(dist_left / length) * (sin_source / sin_intersection);
        assert(result >= -0.1 && result <= 1.1);
    }

    return std::clamp(result, 0.0, 1.0);
}


[[using gnu: hot]]
static double angle_sin(coordinate_t const &direction, coordinate_t forward) {
    forward.rotate_right();
    double product = direction * forward;
    product /= (direction.length() * forward.length());
    assert(product >= -1.01 && product <= 1.01);
    return product;
}

[[using gnu: hot]]
static double angle_sin(coordinate_t const &source, coordinate_t const &right, coordinate_t point) {
    point -= source;
    double product = point * right;
    product /= (point.length() * right.length());
    assert(product >= -1.01 && product <= 1.0);
    return product;
}

[[using gnu: hot]]
static bool orientation_right(coordinate_t const &source, coordinate_t const &right, coordinate_t point) {
    double sin_value = angle_sin(source, right, point);
    return !std::signbit(sin_value);
}

inline bool
is_in_cone(const coordinate_t::component_type direction_left, const coordinate_t::component_type direction_right,
           const coordinate_t::component_type direction_dir) {
    assert((direction_dir == 0.0 || std::isnormal(direction_dir)) && direction_dir >= -std::numbers::pi &&
           direction_dir <= std::numbers::pi);
    assert((direction_left == 0.0 || std::isnormal(direction_left)) && direction_left >= -std::numbers::pi &&
           direction_left <= 2 * std::numbers::pi);
    assert((direction_right == 0.0 || std::isnormal(direction_right)) && direction_right >= -std::numbers::pi &&
           direction_right <= 2 * std::numbers::pi);

    coordinate_t::component_type angle_both{std::fabs(direction_right - direction_left)};
    coordinate_t::component_type angle_l{std::fabs(direction_left - direction_dir)};
    coordinate_t::component_type angle_r{std::fabs(direction_right - direction_dir)};

    angle_both = (angle_both > std::numbers::pi) ? 2 * std::numbers::pi - angle_both : angle_both;
    angle_l = (angle_l > std::numbers::pi) ? 2 * std::numbers::pi - angle_l : angle_l;
    angle_r = (angle_r > std::numbers::pi) ? 2 * std::numbers::pi - angle_r : angle_r;

    assert((angle_both == 0.0 || std::isnormal(angle_both)) && angle_both >= 0.0 && angle_both <= std::numbers::pi);
    assert((angle_l == 0.0 || std::isnormal(angle_l)) && angle_l >= 0.0 && angle_l <= std::numbers::pi);
    assert((angle_r == 0.0 || std::isnormal(angle_r)) && angle_r >= 0.0 && angle_r <= std::numbers::pi);

    angle_both *= 1.0001;
    return (angle_l < angle_both && angle_r < angle_both);
}


inline bool
ignore(const coordinate_t::component_type direction_left, const coordinate_t::component_type direction_right,
       const coordinate_t::component_type direction_dir) {
    return !is_in_cone(direction_left, direction_right, direction_dir);
}


// NEIGHBORS
template<typename Graph, typename Labels, Configuration Config>
steiner_neighbors<Graph, Labels, Config>::node_id_type
steiner_neighbors<Graph, Labels, Config>::min_angle_neighbor_binary_search(const base_edge_id_type &edge_id,
                                                                           const coordinate_t &direction) {
    assert(direction.longitude != 0 || direction.latitude != 0);
    auto &&destination_steiner_info = _graph->steiner_info(edge_id);
    _steiner_point_angle_test_count++;

    // "right"-vector for orientation testing
    coordinate_t target_first = _graph->node_coordinates_first(edge_id);
    coordinate_t target_last  = _graph->node_coordinates_last(edge_id);

    // get direction rotated by 90º to the right
    coordinate_t right = direction;
    right.rotate_right();

    // ensure "right" direction corresponds to higher steiner indices on target edge
    right *= ((target_first - target_last) * right);
    assert(!right.zero());

    // binary search for node with minimal angle using the derivative over the angle depending on steiner index
    using intra_edge_id_type = typename node_id_type::intra_edge_id_type;
    intra_edge_id_type left_index = 1;
    intra_edge_id_type right_index = destination_steiner_info.node_count - 2;
    intra_edge_id_type mid_index = std::clamp(destination_steiner_info.mid_index, (intra_edge_id_type) (left_index + 1),
                                              (intra_edge_id_type) (right_index - 1));

    bool orientation;
    bool norientation;

    steiner_graph::node_id_type destination{edge_id, mid_index};
    assert(mid_index >= 0 && mid_index <= right_index);

    if (left_index >= right_index) [[unlikely]]
        return destination;

    coordinate_t destination_coords;
    // first check
    {
        // get coordinates
        destination_coords = _graph->node_coordinates(destination);

        // compute cos values
        orientation = orientation_right(_source_coordinate, right, destination_coords);
        norientation = !orientation;
    }

    // check which half of the edge is used
    bool const right_half = orientation;
    coordinate_t::component_type const base = (right_half) ? destination_steiner_info.base_second
                                                           : destination_steiner_info.base_first;
    coordinate_t::component_type const ln_base = base;
    // coordinate_t::component_type const ln_base = std::log(base);
    coordinate_t::component_type const log_base_inv = 1 / base;

    left_index = orientation * mid_index + norientation * left_index;
    right_index = norientation * mid_index + orientation * right_index;

    while (right_index - left_index >= 2) [[likely]] {
        // compute m-value,  can possibly be further improved
        intra_edge_id_type step = std::floor(std::log((1 + std::exp(ln_base * (right_index - left_index))) / 2) * log_base_inv);
        assert(step >= 0);
        mid_index = right_half ? (right_index - step) : (left_index + step);
        mid_index = std::clamp(mid_index, (intra_edge_id_type) (left_index + 1), (intra_edge_id_type) (right_index - 1));
        assert (left_index >= right_index || (left_index <= mid_index && mid_index <= right_index));

        // update node ids
        destination.steiner_index = mid_index;

        // get coordinates
        destination_coords = _graph->node_coordinates(destination);
        assert(destination_coords != _source_coordinate);

        // compute cos values
        orientation = orientation_right(_source_coordinate, right, destination_coords);
        norientation = !orientation;

        // update range
        left_index = orientation * mid_index + norientation * left_index;
        right_index = norientation * mid_index + orientation * right_index;

        _steiner_point_angle_test_count++;
    }
    assert(mid_index > 0 && mid_index < destination_steiner_info.node_count - 1);

    return destination;
}
template<typename Graph, typename Labels, Configuration Config>
steiner_neighbors<Graph, Labels, Config>::node_id_type
steiner_neighbors<Graph, Labels, Config>::min_angle_neighbor_atan2(base_edge_id_type edge_id,
                                                                   const coordinate_t &direction) const {
    auto angle_dir{std::atan2(direction)};
    auto left{_graph->node_coordinates_first(edge_id)};
    auto right{_graph->node_coordinates_last(edge_id)};

    auto angle_left = std::atan2(left - _source_coordinate);
    auto angle_right = std::atan2(right - _source_coordinate);
    assert(ignore(angle_left, angle_right, angle_dir) == ignore(angle_right, angle_left, angle_dir));

    if (ignore(angle_left, angle_right, angle_dir)) [[likely]] {
        return optional::none_value<node_id_type>;
    }

    coordinate_t::component_type rel = min_angle_relative_value_atan2(left, right, angle_left, angle_dir);

    if (!is_in_range(rel, 0.0, 1.0 + std::numeric_limits<double>::epsilon()))
        return optional::none_value<node_id_type>;

    return {edge_id, _graph->subdivision_info().index(edge_id, rel)};
}



template<typename Graph, typename Labels, Configuration Config>
steiner_neighbors<Graph, Labels, Config>::node_id_type
steiner_neighbors<Graph, Labels, Config>::min_angle_neighbor_matmul(const base_edge_id_type &edge_id,
                                                                    const coordinate_t &direction) {
    auto rel = min_angle_relative_value_matmul(edge_id, direction);

    if (!is_in_range(rel, 0.0, 1.0 + std::numeric_limits<double>::epsilon()))
        return optional::none_value<node_id_type>;

    return {edge_id, _graph->subdivision_info().index(edge_id, rel)};
}

template<typename Graph, typename Labels, Configuration Config>
template<typename NodeCostPair>
void
steiner_neighbors<Graph, Labels, Config>::add_min_angle_neighbor(const NodeCostPair &node,
                                                                 coordinate_t const &direction,
                                                                 std::vector<NodeCostPair> &out,
                                                                 std::vector<coordinate_t> &out_coordinates) {
    auto const &node_id = node.node();
    auto &&steiner_info = _graph->steiner_info(node_id.edge);

    // face-crossing edges
    for (auto &&edge_id: _graph->base_polyhedron().edges(node.node().edge)) [[likely]] {
        if (edge_id == node.node().edge) [[unlikely]]
            continue;

        // get neighbor
        node_id_type other;
        if constexpr (Configuration::ATAN2 == Config) {
            other = min_angle_neighbor_atan2(edge_id, direction);
        } else if constexpr (Configuration::BINSEARCH == Config) {
            other = min_angle_neighbor_binary_search(edge_id, direction);
        } else if constexpr (Configuration::LINALG == Config) {
            other = min_angle_neighbor_matmul(edge_id, direction);
        }

        if (optional::is_none(other))
            continue;

        assert(other.steiner_index >= 0);
        assert(other.steiner_index < _graph->steiner_info(edge_id).node_count);

        // add point and next steiner point if found
        insert(other, node, out, out_coordinates);

        ++other.steiner_index;

        if (other.steiner_index < steiner_info.node_count) [[likely]] {
            insert(other, node, out, out_coordinates);
        }
    }
}

template<typename Graph, typename Labels, Configuration Config>
template<typename NodeCostPair>
void steiner_neighbors<Graph, Labels, Config>::from_base_node(const NodeCostPair &node, std::vector<NodeCostPair> &out,
                                                              std::vector<coordinate_t> &coordinates_out) {
    _base_node_count++;
    auto const &node_id = node.node();
    auto &&base_node_id = _graph->base_node_id(node_id);

    for (auto &&edge: _graph->base_graph().outgoing_edges(base_node_id)) [[likely]] {
        auto e_id = _graph->base_graph().edge_id(base_node_id, edge.destination);
        steiner_graph::node_id_type destination{e_id, 1};
        // if (destination != node.predecessor()) [[likely]] {
            assert(_graph->has_edge(node_id, destination));
            insert(destination, node, out, coordinates_out);
        // }
    }

    for (auto &&edge: _graph->base_graph().incoming_edges(base_node_id)) [[likely]] {
        auto e_id = _graph->base_graph().edge_id(edge.destination, base_node_id);
        steiner_graph::node_id_type destination(e_id, _graph->steiner_info(e_id).node_count - 2);
        // if (destination != node.predecessor()) [[likely]] {
            assert(_graph->has_edge(node_id, destination));
            insert(destination, node, out, coordinates_out);
        // }
    }

    // should not be necessary, but can be enabled if tree starts to stay on base edges
    // if constexpr (steiner_graph::face_crossing_from_base_nodes) {
    //     // face-crossing edges
    //     auto &&triangle_edges = _graph->base_polyhedron().node_edges(base_node_id);
    //     for (auto base_edge_id: triangle_edges) [[likely]] {
    //         epsilon_spanner(node, base_edge_id, _max_angle_cos, _source_coordinate, out, coordinates_out);
    //     }
    // }

    _base_node_neighbor_count += out.size();
}

template<typename Graph, typename Labels, Configuration Config>
template<typename NodeCostPair>
void steiner_neighbors<Graph, Labels, Config>::from_start_node(const NodeCostPair &node, std::vector<NodeCostPair> &out,
                                                               std::vector<coordinate_t> &coordinates_out) {
    _base_node_count++;
    auto const &node_id = node.node();
    auto &&base_node_id = _graph->base_node_id(node_id);

    for (auto &&edge: _graph->base_graph().outgoing_edges(base_node_id)) [[likely]] {
        auto e_id = _graph->base_graph().edge_id(base_node_id, edge.destination);
        steiner_graph::node_id_type destination{e_id, 1};
        assert(_graph->has_edge(node_id, destination));
        insert(destination, node, out, coordinates_out);
    }

    for (auto &&edge: _graph->base_graph().incoming_edges(base_node_id)) [[likely]] {
        auto e_id = _graph->base_graph().edge_id(edge.destination, base_node_id);
        steiner_graph::node_id_type destination(e_id, _graph->steiner_info(e_id).node_count - 2);
        assert(_graph->has_edge(node_id, destination));
        insert(destination, node, out, coordinates_out);
    }

    // face-crossing edges: make epsilon spanner in all directions
    if constexpr (steiner_graph::face_crossing_from_base_nodes) {
        for (auto &&base_edge_id: _graph->base_polyhedron().node_edges(base_node_id)) [[likely]] {
            epsilon_spanner(node, base_edge_id, -1.0, _source_coordinate, out, coordinates_out);
        }
    }

    _base_node_neighbor_count += out.size();
}

template<typename Graph, typename Labels, Configuration Config>
template<typename NodeCostPair>
void
steiner_neighbors<Graph, Labels, Config>::from_boundary_node(const NodeCostPair &node, std::vector<NodeCostPair> &out,
                                                             std::vector<coordinate_t> &coordinates_out) {
    _boundary_node_count++;
    auto const &node_id = node.node();
    auto &&base_node_id = _graph->base_node_id(node_id);

    for (auto &&edge: _graph->base_graph().outgoing_edges(base_node_id)) [[likely]] {
        auto e_id = _graph->base_graph().edge_id(base_node_id, edge.destination);
        steiner_graph::node_id_type destination{e_id, 1};
        // if (destination != node.predecessor()) [[likely]] {
            assert(_graph->has_edge(node_id, destination));
            insert(destination, node, out, coordinates_out);
        // }
    }

    for (auto &&edge: _graph->base_graph().incoming_edges(base_node_id)) [[likely]] {
        auto e_id = _graph->base_graph().edge_id(edge.destination, base_node_id);
        steiner_graph::node_id_type destination(e_id, _graph->steiner_info(e_id).node_count - 2);
        // if (destination != node.predecessor()) [[likely]] {
            assert(_graph->has_edge(node_id, destination));
            insert(destination, node, out, coordinates_out);
        // }
    }

    if constexpr (steiner_graph::face_crossing_from_base_nodes) {
        // face-crossing edges: make epsilon spanner in all directions
        for (auto &&base_edge_id: _graph->base_polyhedron().node_edges(base_node_id)) [[likely]] {
            epsilon_spanner(node, base_edge_id, -1.0, _source_coordinate, out, coordinates_out);
        }
    }

    _boundary_node_neighbor_count += out.size();
}

template<typename Graph, typename Labels, Configuration Config>
template<typename NodeCostPair>
void
steiner_neighbors<Graph, Labels, Config>::from_steiner_node(const NodeCostPair &node, std::vector<NodeCostPair> &out,
                                                            std::vector<coordinate_t> &out_coordinates) {
    _steiner_point_count++;

    coordinate_t direction{0.0, 0.0};
    if constexpr(HasFaceCrossingPredecessor<NodeCostPair, Graph>) {
        auto &&face_crossing_predecessor = node.face_crossing_predecessor();
        assert(face_crossing_predecessor != node.node());
        assert(_graph->node_coordinates(node.node()) != _graph->node_coordinates(face_crossing_predecessor));
        direction = _source_coordinate - _graph->node_coordinates(face_crossing_predecessor);
    } else if constexpr (HasFaceCrossingPredecessor<typename Labels::value_type, Graph>) {
        auto &&face_crossing_predecessor = (*_labels)[node.node()].face_crossing_predecessor();
        assert(face_crossing_predecessor != node.node());
        assert(_graph->node_coordinates(node.node()) != _graph->node_coordinates(face_crossing_predecessor));
        direction = _source_coordinate - _graph->node_coordinates(face_crossing_predecessor);
    }

    // always add the two neighbors on this edge
    on_edge_neighbors(node, out, out_coordinates);

    // add neighbors that make up steiner interval crossed by ray source-coordinate->direction
    if (!direction.zero()) {
        add_min_angle_neighbor(node, direction, out, out_coordinates);
    }

    _steiner_point_neighbor_count += out.size();
}

template<typename Graph, typename Labels, Configuration Config>
template<typename NodeCostPair>
void steiner_neighbors<Graph, Labels, Config>::epsilon_spanner(const NodeCostPair &node,
                                                               const base_edge_id_type &
                                                               edge_id,
                                                               const coordinate_t::component_type &max_angle_cos,
                                                               const coordinate_t &direction,
                                                               std::vector<NodeCostPair> &out,
                                                               std::vector<coordinate_t> &coordinates_out) {
    auto &&destination_steiner_info = _graph->steiner_info(edge_id);

    node_id_type next{edge_id, destination_steiner_info.mid_index};
    coordinate_t last_direction = direction * -1;
    last_direction.rotate_right();
    for (auto j = next.steiner_index; j >= 1; --j) [[likely]] {
        steiner_graph::node_id_type const destination(edge_id, j);
        coordinate_t const destination_coordinate{_graph->node_coordinates(destination)};
        coordinate_t const new_direction{destination_coordinate - _source_coordinate};

        if (std::abs(angle_sin(new_direction, last_direction)) < _spanner_angle_sin) { [[likely]]
            continue;
        }
        if (angle_cos(direction, new_direction) < max_angle_cos) { [[unlikely]]
            break;
        }

        assert(_graph->has_edge(node.node(), destination));
        insert(destination, node, out, coordinates_out);
        last_direction = new_direction;
    }

    last_direction = direction * -1;
    last_direction.rotate_right();
    for (auto j = next.steiner_index + 1; j < destination_steiner_info.node_count - 1; ++j) [[likely]] {
        steiner_graph::node_id_type const destination(edge_id, j);
        coordinate_t const destination_coordinate{_graph->node_coordinates(destination)};
        coordinate_t const new_direction{destination_coordinate - _source_coordinate};

        if (std::abs(angle_sin(new_direction, last_direction)) < _spanner_angle_sin) { [[likely]]
            continue;
        }
        if (angle_cos(direction, new_direction) < max_angle_cos) { [[unlikely]]
            break;
        }

        assert(_graph->has_edge(node.node(), destination));
        insert(destination, node, out, coordinates_out);
        last_direction = new_direction;
    }
}

template<typename Graph, typename Labels, Configuration Config>
template<typename NodeCostPair>
void
steiner_neighbors<Graph, Labels, Config>::on_edge_neighbors(const NodeCostPair &node, std::vector<NodeCostPair> &out,
                                                            std::vector<coordinate_t> &coordinates_out) {
    auto const &node_id = node.node();
    // auto const &reached_from = node.predecessor();
    auto &&steiner_info = _graph->steiner_info(node_id.edge);

    // for neighboring node on own edge
    if (node_id.steiner_index < steiner_info.node_count - 1) [[likely]] {
        steiner_graph::node_id_type const destination(node_id.edge, node_id.steiner_index + 1);
        // if (destination != reached_from) [[likely]] {
            assert(_graph->has_edge(node_id, destination));
            insert(destination, node, out, coordinates_out);
        // }
    }

    // for other neighboring node on own edge
    if (node_id.steiner_index > 0) [[likely]] {
        steiner_graph::node_id_type const destination(node_id.edge, node_id.steiner_index - 1);
        // if (destination != reached_from) [[likely]] {
            assert(_graph->has_edge(node_id, destination));
            insert(destination, node, out, coordinates_out);
        // }
    }
}

