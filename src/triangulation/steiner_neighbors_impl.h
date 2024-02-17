#include "steiner_neighbors.h"
#include "../graph/base_types.h"
#include "../graph/geometry.h"

#include <cmath>
#include <numbers>
#include <vector>
#include <algorithm>

template<typename Graph, typename Labels>
template<typename NodeCostPair>
requires HasFaceCrossingPredecessor<NodeCostPair, Graph>
typename Graph::node_id_type const &steiner_neighbors<Graph, Labels>::find_face_crossing_predecessor( const NodeCostPair &node) {
    assert(!is_none(node.face_crossing_predecessor()));
    return node.face_crossing_predecessor();
}

template<typename Graph, typename Labels>
template<typename NodeCostPair>
requires (!HasFaceCrossingPredecessor<NodeCostPair, Graph>)
typename Graph::node_id_type
steiner_neighbors<Graph, Labels>::find_face_crossing_predecessor(const NodeCostPair &node) const {
    auto const &node_id = node.node();
    auto const &reached_from = node.predecessor();

    // find first predecessor on different edge
    auto closer_face_crossing_predecessor = node_id;
    auto face_crossing_predecessor = reached_from;
    while ((face_crossing_predecessor.edge == node_id.edge ||
            _graph.is_base_node(closer_face_crossing_predecessor))
           && face_crossing_predecessor != closer_face_crossing_predecessor
            ) [[likely]] {
        closer_face_crossing_predecessor = face_crossing_predecessor;
        face_crossing_predecessor = _labels.at(face_crossing_predecessor).predecessor();
    }
    return face_crossing_predecessor;
}

template<typename Graph, typename Labels>
template<typename NodeCostPair>
void steiner_neighbors<Graph, Labels>::operator()(const NodeCostPair &node, std::vector<NodeCostPair> &out) {
    auto const &node_id = node.node();
    assert(!is_none(node_id));

    _source_coordinate = _graph.node_coordinates(node_id);

    //
    bool const is_start_node = node_id == node.predecessor();
    bool const is_base_node = _graph.is_base_node(node_id);
    bool is_boundary_node = false;

    if constexpr (HasFaceCrossingPredecessor<NodeCostPair, Graph>) {
        assert(is_start_node ||
               (!is_none(node.face_crossing_predecessor()) && node.face_crossing_predecessor() != node.node()));
    }

    if (!is_base_node && !is_start_node) [[likely]] {
        from_steiner_node(node, out);
    } else if (!is_base_node) {
        from_start_node(node, out);
    } else {
        auto base_node_id = _graph.base_node_id(node_id);
        is_boundary_node = _graph.is_boundary_node(base_node_id);
        if (is_boundary_node) { [[unlikely]]
            from_boundary_node(node, out);
        } else {
            from_base_node(node, out);
        }
    }

    // set face crossing predecessor of neighbors
    if constexpr (HasFaceCrossingPredecessor<NodeCostPair, Graph>) {
        // if this node is a boundary node, set it as the new face crossing predecessor
        auto fcp = (is_none(node.face_crossing_predecessor()) || is_boundary_node)
                   ? node.node() : node.face_crossing_predecessor();

        assert(!is_none(fcp));
        for (int e = 0; e < out.size(); ++e) [[likely]] {
            out[e].face_crossing_predecessor() = fcp;
        }
    }

    // compute distances (can be vectorized)
    assert(out.size() == _destination_coordinates.size());
    for (int e = 0; e < out.size(); ++e) [[likely]] {
        out[e].distance() += distance(_source_coordinate, _destination_coordinates[e]);
        assert(out[e].distance() > 0);
    }

    _destination_coordinates.clear();
}

template<typename Graph, typename Labels>
coordinate_t::component_type steiner_neighbors<Graph, Labels>::min_angle_relative_value(base_edge_id_type edge_id,
                                                                                        coordinate_t const &direction) const {
    // src->left, right-left
    coordinate_t coordinates[2];
    {
        coordinate_t const src = _graph.node_coordinates(_graph.base_graph().source(edge_id));
        coordinates[1] = src - _source_coordinate;
        coordinates[0] = src - _graph.node_coordinates(_graph.base_graph().destination(edge_id));
    }

    coordinate_t::component_type angle_source;
    coordinate_t::component_type angle_left;
    {
        coordinate_t::component_type angles[3];
        angles[0] = std::atan2(coordinates[1]);
        angles[1] = std::atan2(coordinates[0]);
        angles[2] = std::atan2(direction);
        // atan2_approximation(3, coordinates, angles);

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
        coordinate_t::component_type const dist_left = coordinates[1].sqr_length();
        coordinate_t::component_type const length = coordinates[0].sqr_length();

        // compute sin values
        coordinate_t::component_type const sin_source = std::sin(angle_source);

        // angle between intersection->left and intersection->src
        // std::sin(std::numbers::pi - angle_source - angle_left) is equal to:
        coordinate_t::component_type const sin_intersection = std::sin(angle_source + angle_left);

        // found using law of sines
        result = std::sqrt(dist_left / length) * (sin_source / sin_intersection);
        assert(result >= -0.00001 && result <= 1.00001);
    }

    return std::clamp(result, 0.0, 1.0);
}

template<typename Graph, typename Labels>
coordinate_t::component_type steiner_neighbors<Graph, Labels>::min_angle_relative_value(base_edge_id_type edge_id,
                                                                                        coordinate_t left,
                                                                                        coordinate_t right,
                                                                                        coordinate_t::component_type const direction_left,
                                                                                        coordinate_t::component_type const direction_dir) const {
    // left->right
    right = left - right;
    // source->left
    left  -= _source_coordinate;

    coordinate_t::component_type angle_source;
    coordinate_t::component_type angle_left;
    {
        // angle between src->left and src->intersection
        angle_source = std::fabs(direction_left - direction_dir);

        // angle between src->left and right->left
        angle_left   = std::fabs(direction_left - std::atan2(right));

        //
        angle_source = (angle_source > std::numbers::pi) ? (2 * std::numbers::pi) - angle_source : angle_source;
        angle_left   = (angle_left   > std::numbers::pi) ? (2 * std::numbers::pi) - angle_left   : angle_left;
    }

    coordinate_t::component_type result;
    {
        // side lengths
        coordinate_t::component_type const dist_left = left.sqr_length();
        coordinate_t::component_type const length    = right.sqr_length();

        // compute sin values
        coordinate_t::component_type const sin_source = std::sin(angle_source);

        // angle between intersection->left and intersection->src
        // std::sin(std::numbers::pi - angle_source - angle_left) is equal to:
        coordinate_t::component_type const sin_intersection = std::sin(angle_source + angle_left);

        // found using law of sines
        result = std::sqrt(dist_left / length) * (sin_source / sin_intersection);
        assert(result >= -0.01 && result <= 1.01);
    }

    return std::clamp(result, 0.0, 1.0);
}



template<typename Graph, typename Labels>
template<typename NodeCostPair>
void steiner_neighbors<Graph, Labels>::from_base_node(const NodeCostPair &node, std::vector<NodeCostPair> &out) {
    _base_node_count++;
    auto const &node_id = node.node();
    auto &&base_node_id = _graph.base_node_id(node_id);

    for (auto &&e: _graph.base_graph().outgoing_edges(base_node_id)) [[likely]] {
        auto e_id = _graph.base_graph().edge_id(base_node_id, e.destination);
        steiner_graph::node_id_type destination{e_id, 1};
        if (destination != node.predecessor()) [[likely]] {
            assert(_graph.has_edge(node_id, destination));
            out.emplace_back(destination, node_id, node.distance());
            _destination_coordinates.emplace_back(_graph.node_coordinates(destination));
        }
    }

    for (auto &&e: _graph.base_graph().incoming_edges(base_node_id)) [[likely]] {
        auto e_id = _graph.base_graph().edge_id(e.destination, base_node_id);
        steiner_graph::node_id_type destination(e_id, _graph.steiner_info(e_id).node_count - 2);
        if (destination != node.predecessor()) [[likely]] {
            assert(_graph.has_edge(node_id, destination));
            out.emplace_back(destination, node.node(), node.distance());
            _destination_coordinates.emplace_back(_graph.node_coordinates(destination));
        }
    }

    // should not be necessary, but can be enabled if tree starts to stay on base edges
    // if constexpr (steiner_graph::face_crossing_from_base_nodes) {
    //     // face-crossing edges
    //     auto &&triangle_edges = _graph.base_polyhedron().node_edges(base_node_id);
    //     for (auto base_edge_id: triangle_edges) [[likely]] {
    //         epsilon_spanner(node, base_edge_id, _max_angle_cos, _source_coordinate, out);
    //     }
    // }

    _base_node_neighbor_count += out.size();
}

template<typename Graph, typename Labels>
template<typename NodeCostPair>
void steiner_neighbors<Graph, Labels>::from_start_node(const NodeCostPair &node, std::vector<NodeCostPair> &out) {
    _base_node_count++;
    auto const &node_id = node.node();
    auto &&base_node_id = _graph.base_node_id(node_id);

    for (auto &&e: _graph.base_graph().outgoing_edges(base_node_id)) [[likely]] {
        auto e_id = _graph.base_graph().edge_id(base_node_id, e.destination);
        steiner_graph::node_id_type destination{e_id, 1};
        assert(_graph.has_edge(node_id, destination));
        out.emplace_back(destination, node_id, node.distance());
        _destination_coordinates.emplace_back(_graph.node_coordinates(destination));
    }

    for (auto &&e: _graph.base_graph().incoming_edges(base_node_id)) [[likely]] {
        auto e_id = _graph.base_graph().edge_id(e.destination, base_node_id);
        steiner_graph::node_id_type destination(e_id, _graph.steiner_info(e_id).node_count - 2);
        assert(_graph.has_edge(node_id, destination));
        out.emplace_back(destination, node.node(), node.distance());
        _destination_coordinates.emplace_back(_graph.node_coordinates(destination));
    }

    // face-crossing edges: make epsilon spanner in all directions
    if constexpr (steiner_graph::face_crossing_from_base_nodes) {
        for (auto &&base_edge_id: _graph.base_polyhedron().node_edges(base_node_id)) [[likely]] {
            epsilon_spanner(node, base_edge_id, -1.0, _source_coordinate, out);
        }
    }

    _base_node_neighbor_count += out.size();
}

template<typename Graph, typename Labels>
template<typename NodeCostPair>
void steiner_neighbors<Graph, Labels>::from_boundary_node(const NodeCostPair &node, std::vector<NodeCostPair> &out) {
    _boundary_node_count++;
    auto const &node_id = node.node();
    auto &&base_node_id = _graph.base_node_id(node_id);

    for (auto &&e: _graph.base_graph().outgoing_edges(base_node_id)) [[likely]] {
        auto e_id = _graph.base_graph().edge_id(base_node_id, e.destination);
        steiner_graph::node_id_type destination{e_id, 1};
        if (destination != node.predecessor()) [[likely]] {
            assert(_graph.has_edge(node_id, destination));
            out.emplace_back(destination, node_id, node.distance());
            _destination_coordinates.emplace_back(_graph.node_coordinates(destination));
        }
    }

    for (auto &&e: _graph.base_graph().incoming_edges(base_node_id)) [[likely]] {
        auto e_id = _graph.base_graph().edge_id(e.destination, base_node_id);
        steiner_graph::node_id_type destination(e_id, _graph.steiner_info(e_id).node_count - 2);
        if (destination != node.predecessor()) [[likely]] {
            assert(_graph.has_edge(node_id, destination));
            out.emplace_back(destination, node.node(), node.distance());
            _destination_coordinates.emplace_back(_graph.node_coordinates(destination));
        }
    }

    if constexpr (steiner_graph::face_crossing_from_base_nodes) {
        // face-crossing edges: make epsilon spanner in all directions
        for (auto &&base_edge_id: _graph.base_polyhedron().node_edges(base_node_id)) [[likely]] {
            epsilon_spanner(node, base_edge_id, -1.0, _source_coordinate, out);
        }
    }

    _boundary_node_neighbor_count += out.size();
}

inline bool is_in_cone(const coordinate_t::component_type direction_left, const coordinate_t::component_type direction_right, const coordinate_t::component_type direction_dir) {
    assert((direction_dir == 0.0 || std::isnormal(direction_dir)) && direction_dir >= -std::numbers::pi && direction_dir <= std::numbers::pi);
    assert((direction_left  == 0.0 || std::isnormal(direction_left )) && direction_left  >= -std::numbers::pi && direction_left  <= 2 * std::numbers::pi);
    assert((direction_right == 0.0 || std::isnormal(direction_right)) && direction_right >= - std::numbers::pi && direction_right <= 2 * std::numbers::pi);

    coordinate_t::component_type angle_both { std::fabs(direction_right - direction_left) };
    coordinate_t::component_type angle_l    { std::fabs(direction_left  - direction_dir ) };
    coordinate_t::component_type angle_r    { std::fabs(direction_right - direction_dir ) };

    angle_both = (angle_both > std::numbers::pi) ? 2 * std::numbers::pi - angle_both : angle_both;
    angle_l    = (angle_l    > std::numbers::pi) ? 2 * std::numbers::pi - angle_l    : angle_l;
    angle_r    = (angle_r    > std::numbers::pi) ? 2 * std::numbers::pi - angle_r    : angle_r;

    assert((angle_both == 0.0 || std::isnormal(angle_both)) && angle_both >= 0.0 && angle_both <= std::numbers::pi);
    assert((angle_l  == 0.0 || std::isnormal(angle_l )) && angle_l  >= 0.0 && angle_l  <= std::numbers::pi);
    assert((angle_r == 0.0 || std::isnormal(angle_r)) && angle_r >= 0.0 && angle_r <= std::numbers::pi);

    return (angle_l < angle_both && angle_r < angle_both);
}
inline bool ignore(const coordinate_t::component_type direction_left, const coordinate_t::component_type direction_right, const coordinate_t::component_type direction_dir) {
    return !is_in_cone(direction_left, direction_right, direction_dir);
}

template<typename Graph, typename Labels>
template<typename NodeCostPair>
void steiner_neighbors<Graph, Labels>::from_steiner_node(const NodeCostPair &node, std::vector<NodeCostPair> &out) {
    _steiner_point_count++;
    auto const &node_id = node.node();

    auto &&face_crossing_predecessor = find_face_crossing_predecessor(node);
    coordinate_t direction = _source_coordinate - _graph.node_coordinates(face_crossing_predecessor);
    coordinate_t::component_type angle_dir = std::atan2(direction);

    assert(face_crossing_predecessor != node_id);
    assert(_graph.node_coordinates(node_id) != _graph.node_coordinates(face_crossing_predecessor));
    assert(direction.latitude != 0.0 || direction.longitude != 0.0);

    auto &&steiner_info = _graph.steiner_info(node_id.edge);

    // face-crossing edges
    for (auto &&base_edge_id: _graph.base_polyhedron().edges(node.node().edge)) [[likely]] {
        auto left  { _graph.node_coordinates(_graph.base_graph().source(base_edge_id)) };
        auto right { _graph.node_coordinates(_graph.base_graph().destination(base_edge_id)) };

        auto angle_left  = std::atan2(left  - _source_coordinate);
        auto angle_right = std::atan2(right - _source_coordinate);
        assert(ignore(angle_left, angle_right, angle_dir) == ignore(angle_right, angle_left, angle_dir));
        assert(ignore(angle_left, angle_dir, angle_right) || ignore(angle_left, angle_right, angle_dir));
        if (ignore(angle_left, angle_right, angle_dir)) [[likely]] {
            continue;
        }

        add_min_angle_neighbor(node, base_edge_id, left, right, angle_left, angle_dir, out);
        add_min_angle_neighbor(node, base_edge_id, direction, out);

        // required in paper, but probably not on unweighted triangulations
        // if (face_crossing_predecessor != reached_from && _source_coordinate != _graph.node_coordinates(reached_from)) [[unlikely]] {
        //     assert(_graph.node_coordinates(reached_from) != _source_coordinate);
        //     add_min_angle_neighbor(node, base_edge_id, _max_angle_cos, _source_coordinate - _graph.node_coordinates(reached_from), out);
        // }
    }

    // always add the two neighbors on this edge
    on_edge_neighbors(node, out);

    _steiner_point_neighbor_count += out.size();
}

template<typename Graph, typename Labels>
template<typename NodeCostPair>
void steiner_neighbors<Graph, Labels>::epsilon_spanner(const NodeCostPair &node,
                                                       const base_edge_id_type &
                                                       edge_id,
                                                       const coordinate_t::component_type &max_angle_cos,
                                                       const coordinate_t &direction,
                                                       std::vector<NodeCostPair> &out) {
    auto const &node_id = node.node();
    auto &&destination_steiner_info = _graph.steiner_info(edge_id);

    node_id_type next{edge_id, destination_steiner_info.mid_index};
    coordinate_t last_direction = direction * -1;
    for (auto j = next.steiner_index; j >= 1; --j) [[likely]] {
        steiner_graph::node_id_type const destination(edge_id, j);
        coordinate_t const destination_coordinate{_graph.node_coordinates(destination)};
        coordinate_t const new_direction{destination_coordinate - _source_coordinate};

        if (angle_cos(direction, new_direction) < max_angle_cos) [[unlikely]]
            break;
        if (angle_cos(last_direction, new_direction) > _spanner_angle_cos) [[likely]]
            continue;

        assert(_graph.has_edge(node_id, destination));
        out.emplace_back(destination, node_id, node.distance());
        _destination_coordinates.emplace_back(destination_coordinate);
        last_direction = new_direction;
    }

    last_direction = direction * -1;
    for (auto j = next.steiner_index + 1; j < destination_steiner_info.node_count - 1; ++j) [[likely]] {
        steiner_graph::node_id_type const destination(edge_id, j);
        coordinate_t const destination_coordinate{_graph.node_coordinates(destination)};
        coordinate_t const new_direction{destination_coordinate - _source_coordinate};

        if (angle_cos(direction, new_direction) < max_angle_cos) [[unlikely]]
            break;
        if (angle_cos(last_direction, new_direction) > _spanner_angle_cos) [[likely]]
            continue;

        assert(_graph.has_edge(node_id, destination));
        out.emplace_back(destination, node_id, node.distance());
        _destination_coordinates.emplace_back(destination_coordinate);
        last_direction = new_direction;
    }
}

template<typename Graph, typename Labels>
template<typename NodeCostPair>
void
steiner_neighbors<Graph, Labels>::add_min_angle_neighbor(const NodeCostPair &node,
                                                         const base_edge_id_type &edge_id,
                                                         coordinate_t const& left,
                                                         coordinate_t const& right,
                                                         coordinate_t::component_type const angle_left,
                                                         coordinate_t::component_type const angle_dir,
                                                         std::vector<NodeCostPair> &out) {
    coordinate_t::component_type rel = min_angle_relative_value(edge_id, left, right, angle_left, angle_dir);
    node_id_type other{edge_id, _graph.subdivision_info().index(edge_id, rel)};

    {
        coordinate_t destination_coordinate{_graph.node_coordinates(other)};
        out.emplace_back(other, node.node(), node.distance());
        _destination_coordinates.emplace_back(destination_coordinate);
    }

    ++other.steiner_index;

    {
        coordinate_t destination_coordinate{_graph.node_coordinates(other)};
        out.emplace_back(other, node.node(), node.distance());
        _destination_coordinates.emplace_back(destination_coordinate);
    }
}


template<typename Graph, typename Labels>
template<typename NodeCostPair>
void
steiner_neighbors<Graph, Labels>::add_min_angle_neighbor(const NodeCostPair &node,
                                                         const base_edge_id_type &edge_id,
                                                         const coordinate_t &direction,
                                                         std::vector<NodeCostPair> &out) {
    assert(direction.longitude != 0 || direction.latitude != 0);
    //node_id_type next;
    //coordinate_t::component_type cos1, cos2;
    // if constexpr (HasSuccessorHint<typename Labels::edge_label_type, Graph>) {
    //     auto& hint = _labels.at(node.node().edge).successor_hint();

    //     if (hint.edge == edge_id) { // valid hint
    //         // search for best neighbor here
    //         next = find_min_angle_neighbors_hinted(edge_id, direction, hint, cos1, cos2);
    //     } else {
    //         next = find_min_angle_neighbors(edge_id, direction, cos1, cos2);
    //     }

    //     hint = next;
    // } else {
    //     next = find_min_angle_neighbors(edge_id, direction, cos1, cos2);
    // }

    coordinate_t::component_type rel = min_angle_relative_value(edge_id, direction);
    node_id_type other{edge_id, _graph.subdivision_info().index(edge_id, rel)};

    {
        coordinate_t destination_coordinate{_graph.node_coordinates(other)};
        out.emplace_back(other, node.node(), node.distance());
        _destination_coordinates.emplace_back(destination_coordinate);
    }

    ++other.steiner_index;

    {
        coordinate_t destination_coordinate{_graph.node_coordinates(other)};
        out.emplace_back(other, node.node(), node.distance());
        _destination_coordinates.emplace_back(destination_coordinate);
    }
}


template<typename Graph, typename Labels>
steiner_neighbors<Graph, Labels>::node_id_type
steiner_neighbors<Graph, Labels>::find_min_angle_neighbors(const base_edge_id_type &edge_id,
                                                           const coordinate_t &direction,
                                                           coordinate_t::component_type &cos,
                                                           coordinate_t::component_type &cos_n) {
    assert(direction.longitude != 0 || direction.latitude != 0);
    auto &&destination_steiner_info = _graph.steiner_info(edge_id);
    _steiner_point_angle_test_count++;

    // binary search for node with minimal angle using the derivative over the angle depending on steiner index
    using intra_edge_id_type = typename node_id_type::intra_edge_id_type;
    intra_edge_id_type l = 1;
    intra_edge_id_type r = destination_steiner_info.node_count - 2;
    intra_edge_id_type m = std::clamp(destination_steiner_info.mid_index, (intra_edge_id_type) (l + 1),
                                      (intra_edge_id_type) (r - 1));
    coordinate_t::component_type cos_next = 1.0;
    coordinate_t::component_type cos_current = 1.0;

    steiner_graph::node_id_type destination{edge_id, m};
    steiner_graph::node_id_type destination_next{edge_id, m + 1};
    assert(m >= 0 && m <= r);

    if (l >= r) [[unlikely]]
        return destination;

    coordinate_t direction_next;
    coordinate_t direction_current;
    // first check
    {
        // get coordinates
        direction_next = _graph.node_coordinates(destination_next);
        direction_current = _graph.node_coordinates(destination);

        // make directions from source
        assert(direction_next != _source_coordinate);
        assert(direction_current != _source_coordinate);
        direction_next -= _source_coordinate;
        direction_current -= _source_coordinate;

        // compute cos values (can hopefully be vectorized)
        cos_next = angle_cos(direction, direction_next);
        cos_current = angle_cos(direction, direction_current);
        assert(cos_current == 0 || std::isnormal(cos_current));
    }

    // check which half of the edge is used
    bool const right_half = (cos_next > cos_current);
    coordinate_t::component_type const base = (right_half) ? destination_steiner_info.base_second
                                                           : destination_steiner_info.base_first;
    coordinate_t::component_type const ln_base = base;
    // coordinate_t::component_type const ln_base = std::log(base);
    coordinate_t::component_type const log_base_inv = 1 / base;

    bool right = right_half, left = !right_half;
    l = right * m + left * l;
    r = left * m + right * r;

    while (r - l >= 2) [[likely]] {
        // compute m-value,  can possibly be further improved
        intra_edge_id_type step = std::floor(std::log((1 + std::exp(ln_base * (r - l))) / 2) * log_base_inv);
        assert(step >= 0);
        m = right_half ? (r - step) : (l + step);
        m = std::clamp(m, (intra_edge_id_type) (l + 1), (intra_edge_id_type) (r - 1));
        assert (l >= r || (l <= m && m <= r));

        // update node ids
        destination_next.steiner_index = m + 1;
        destination.steiner_index = m;

        // get coordinates
        direction_next = _graph.node_coordinates(destination_next);
        direction_current = _graph.node_coordinates(destination);
        assert(direction_next != _source_coordinate);
        assert(direction_current != _source_coordinate);

        // make directions from source
        direction_next -= _source_coordinate;
        direction_current -= _source_coordinate;
        assert(!direction.zero() && !direction_current.zero());

        // compute cos values (can hopefully be vectorized)
        cos_next = angle_cos(direction, direction_next);
        cos_current = angle_cos(direction, direction_current);
        assert(cos_next == 0 || std::isnormal(cos_next));
        assert(cos_current == 0 || std::isnormal(cos_current));

        // update range
        right = (cos_next > cos_current);
        left = !right;
        l = right * m + left * l;
        r = left * m + right * r;

        _steiner_point_angle_test_count++;
    }
    assert(m > 0 && m < destination_steiner_info.node_count - 1);

    // store cos value
    cos = cos_current;
    cos_n = cos_next;
    return destination;
}

template<typename Graph, typename Labels>
template<typename NodeCostPair>
void steiner_neighbors<Graph, Labels>::on_edge_neighbors(const NodeCostPair &node, std::vector<NodeCostPair> &out) {
    auto const &node_id = node.node();
    auto const &reached_from = node.predecessor();
    auto &&steiner_info = _graph.steiner_info(node_id.edge);

    // for neighboring node on own edge
    if (node_id.steiner_index < steiner_info.node_count - 1) [[likely]] {
        steiner_graph::node_id_type const destination(node_id.edge, node_id.steiner_index + 1);
        if (destination != reached_from) [[likely]] {
            coordinate_t destination_coordinate{_graph.node_coordinates(destination)};
            assert(_graph.has_edge(node_id, destination));
            out.emplace_back(destination, node_id, node.distance());
            _destination_coordinates.emplace_back(destination_coordinate);
        }
    }

    // for other neighboring node on own edge
    if (node_id.steiner_index > 0) [[likely]] {
        steiner_graph::node_id_type const destination(node_id.edge, node_id.steiner_index - 1);
        if (destination != reached_from) [[likely]] {
            coordinate_t destination_coordinate{_graph.node_coordinates(destination)};
            assert(_graph.has_edge(node_id, destination));
            out.emplace_back(destination, node_id, node.distance());
            _destination_coordinates.emplace_back(destination_coordinate);
        }
    }
}

template<typename Graph, typename Labels>
typename steiner_neighbors<Graph, Labels>::node_id_type steiner_neighbors<Graph, Labels>::
find_min_angle_neighbors_hinted(base_edge_id_type const &edge_id, coordinate_t const &direction,
                                node_id_type const &hint, coordinate_t::component_type &cos,
                                coordinate_t::component_type &cos2) {
    return find_min_angle_neighbors(edge_id, direction, cos, cos2);
}


