#include "steiner_neighbors.h"
#include "../graph/base_types.h"

#include <cmath>
#include <vector>

template<typename Graph, typename Labels>
template<typename NodeCostPair> requires HasFaceCrossingPredecessor<NodeCostPair, Graph>
typename Graph::node_id_type const& steiner_neighbors<Graph, Labels>::find_face_crossing_predecessor(
    const NodeCostPair&node) {
    assert(!is_none(node.face_crossing_predecessor()));
    return node.face_crossing_predecessor();
}

template<typename Graph, typename Labels>
template<typename NodeCostPair> requires (!HasFaceCrossingPredecessor<NodeCostPair, Graph>)
typename Graph::node_id_type steiner_neighbors<Graph, Labels>::find_face_crossing_predecessor(const NodeCostPair&node) {
    auto const&node_id = node.node();
    auto const&reached_from = node.predecessor();

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
void steiner_neighbors<Graph, Labels>::operator()(const NodeCostPair&node, std::vector<NodeCostPair>&out) {
    auto const& node_id = node.node();
    assert(!is_none(node_id));

    _source_coordinate = _graph.node(node_id).coordinates;

    //
    bool is_start_node = node_id == node.predecessor();
    bool is_base_node = _graph.is_base_node(node_id);
    bool is_boundary_node = false;

    if constexpr (HasFaceCrossingPredecessor<NodeCostPair, Graph>) {
        assert( is_start_node || (!is_none(node.face_crossing_predecessor()) && node.face_crossing_predecessor() != node. node()));
    }


    if (is_start_node) [[unlikely]] {
        from_start_node(node, out);
    } else if (is_base_node) [[unlikely]] {
        auto base_node_id = _graph.base_node_id(node_id);
        is_boundary_node = _graph.is_boundary_node(base_node_id);
        if (is_boundary_node) { [[unlikely]]
            from_boundary_node(node, out);
        } else
            from_base_node(node, out);
    } else {
        from_steiner_node(node, out);
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

        for (int e = 0; e < out.size(); ++e) [[likely]] {
            out[e].face_crossing_predecessor() = fcp;
            if (out[e].face_crossing_predecessor() == out[e].node()) [[unlikely]]
                out[e].face_crossing_predecessor() = node.node();
            assert(out[e].face_crossing_predecessor() != out[e].node());
            assert(out[e].predecessor() != out[e].node());
            assert(!is_none(out[e].face_crossing_predecessor()));
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
template<typename NodeCostPair>
void steiner_neighbors<Graph, Labels>::from_base_node(const NodeCostPair&node, std::vector<NodeCostPair>&out) {
    _base_node_count++;
    auto const&node_id = node.node();
    auto&&base_node_id = _graph.base_node_id(node_id);

    for (auto&&e: _graph.base_graph().outgoing_edges(base_node_id)) [[likely]] {
        auto e_id = _graph.base_graph().edge_id(base_node_id, e.destination);
        steiner_graph::node_id_type destination{e_id, 1};
        if (destination != node.predecessor()) [[likely]] {
            assert(_graph.has_edge(node_id, destination));
            out.emplace_back(destination, node_id, node.distance());
            _destination_coordinates.emplace_back(_graph.node(destination).coordinates);
        }
    }

    for (auto&&e: _graph.base_graph().incoming_edges(base_node_id)) [[likely]] {
        auto e_id = _graph.base_graph().edge_id(e.destination, base_node_id);
        steiner_graph::node_id_type destination(e_id, _graph.steiner_info(e_id).node_count - 2);
        if (destination != node.predecessor()) [[likely]] {
            assert(_graph.has_edge(node_id, destination));
            out.emplace_back(destination, node.node(), node.distance());
            _destination_coordinates.emplace_back(_graph.node(destination).coordinates);
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
void steiner_neighbors<Graph, Labels>::from_start_node(const NodeCostPair&node, std::vector<NodeCostPair>&out) {
    _base_node_count++;
    auto const&node_id = node.node();
    auto&&base_node_id = _graph.base_node_id(node_id);

    for (auto&&e: _graph.base_graph().outgoing_edges(base_node_id)) [[likely]] {
        auto e_id = _graph.base_graph().edge_id(base_node_id, e.destination);
        steiner_graph::node_id_type destination{e_id, 1};
        if (destination != node.predecessor()) [[likely]] {
            assert(_graph.has_edge(node_id, destination));
            out.emplace_back(destination, node_id, node.distance());
            _destination_coordinates.emplace_back(_graph.node(destination).coordinates);
        }
    }

    for (auto&&e: _graph.base_graph().incoming_edges(base_node_id)) [[likely]] {
        auto e_id = _graph.base_graph().edge_id(e.destination, base_node_id);
        steiner_graph::node_id_type destination(e_id, _graph.steiner_info(e_id).node_count - 2);
        if (destination != node.predecessor()) [[likely]] {
            assert(_graph.has_edge(node_id, destination));
            out.emplace_back(destination, node.node(), node.distance());
            _destination_coordinates.emplace_back(_graph.node(destination).coordinates);
        }
    }

    // face-crossing edges: make epsilon spanner in all directions
    if constexpr (steiner_graph::face_crossing_from_base_nodes) {
        auto&&triangle_edges = _graph.base_polyhedron().node_edges(base_node_id);
        for (auto&&base_edge_id: triangle_edges) [[likely]] {
            epsilon_spanner(node, base_edge_id, -1.0, _source_coordinate, out);
        }
    }

    _base_node_neighbor_count += out.size();
}

template<typename Graph, typename Labels>
template<typename NodeCostPair>
void steiner_neighbors<Graph, Labels>::from_boundary_node(const NodeCostPair&node, std::vector<NodeCostPair>&out) {
    _boundary_node_count++;
    auto const&node_id = node.node();
    auto&&base_node_id = _graph.base_node_id(node_id);

    for (auto&&e: _graph.base_graph().outgoing_edges(base_node_id)) [[likely]] {
        auto e_id = _graph.base_graph().edge_id(base_node_id, e.destination);
        steiner_graph::node_id_type destination{e_id, 1};
        if (destination != node.predecessor()) [[likely]] {
            assert(_graph.has_edge(node_id, destination));
            out.emplace_back(destination, node_id, node.distance());
            _destination_coordinates.emplace_back(_graph.node(destination).coordinates);
        }
    }

    for (auto&&e: _graph.base_graph().incoming_edges(base_node_id)) [[likely]] {
        auto e_id = _graph.base_graph().edge_id(e.destination, base_node_id);
        steiner_graph::node_id_type destination(e_id, _graph.steiner_info(e_id).node_count - 2);
        if (destination != node.predecessor()) [[likely]] {
            assert(_graph.has_edge(node_id, destination));
            out.emplace_back(destination, node.node(), node.distance());
            _destination_coordinates.emplace_back(_graph.node(destination).coordinates);
        }
    }

    if constexpr (steiner_graph::face_crossing_from_base_nodes) {
        // face-crossing edges: make epsilon spanner in all directions
        for (auto&&base_edge_id: _graph.base_polyhedron().node_edges(base_node_id)) [[likely]] {
            epsilon_spanner(node, base_edge_id, -1.0, _source_coordinate, out);
        }
    }

    _boundary_node_neighbor_count += out.size();
}

template<typename Graph, typename Labels>
template<typename NodeCostPair>
void steiner_neighbors<Graph, Labels>::from_steiner_node(const NodeCostPair&node, std::vector<NodeCostPair>&out) {
    _steiner_point_count++;
    auto const&node_id = node.node();

    // always add the two neighbors on this edge
    on_edge_neighbors(node, out);

    auto&&face_crossing_predecessor = find_face_crossing_predecessor(node);
    coordinate_t direction = _source_coordinate - _graph.node(face_crossing_predecessor).coordinates;

    // normalize direction length so that it has the same length as the current edge (to prevent numerical issues)
    direction *= distance(_graph.node(_graph.base_graph().source(node.node().edge)).coordinates,
                          _graph.node(_graph.base_graph().destination(node.node().edge)).coordinates) / direction.length();

    assert(face_crossing_predecessor != node_id);
    assert(_graph.node(node_id).coordinates != _graph.node(face_crossing_predecessor).coordinates);
    assert(direction.latitude != 0.0 || direction.longitude != 0.0);

    auto&&steiner_info = _graph.steiner_info(node_id.edge);

    // face-crossing edges
    for (auto&&base_edge_id: _graph.base_polyhedron().edges(node.node().edge)) [[likely]] {
        if (ignore_edge(base_edge_id, direction)) [[unlikely]]
            continue;

        add_min_angle_neighbor(node, base_edge_id, _max_angle_cos, direction, out);

        // required in paper, but probably not on unweighted triangulations
        // if (face_crossing_predecessor != reached_from && _source_coordinate != _graph.node(reached_from).coordinates) [[unlikely]] {
        //     assert(_graph.node(reached_from).coordinates != _source_coordinate);
        //     add_min_angle_neighbor(node, base_edge_id, _max_angle_cos, _source_coordinate - _graph.node(reached_from).coordinates, out);
        // }
    }


    _steiner_point_neighbor_count += out.size();
}

template<typename Graph, typename Labels>
template<typename NodeCostPair>
void steiner_neighbors<Graph, Labels>::epsilon_spanner(const NodeCostPair&node,
                                                       const base_edge_id_type&
                                                       edge_id,
                                                       const double&max_angle_cos, const coordinate_t&direction,
                                                       std::vector<NodeCostPair>&out) {
    auto const&node_id = node.node();
    auto&&destination_steiner_info = _graph.steiner_info(edge_id);

    node_id_type next{edge_id, destination_steiner_info.mid_index};
    coordinate_t last_direction = direction * -1;
    for (auto j = next.steiner_index; j >= 1; --j) [[likely]] {
        steiner_graph::node_id_type const destination(edge_id, j);
        coordinate_t const destination_coordinate{_graph.node(destination).coordinates};
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
        coordinate_t const destination_coordinate{_graph.node(destination).coordinates};
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
steiner_neighbors<Graph, Labels>::add_min_angle_neighbor(const NodeCostPair&node,
                                                         const base_edge_id_type& edge_id,
                                                         const double&max_angle_cos, const coordinate_t&direction,
                                                         std::vector<NodeCostPair>&out) {
    assert(direction.longitude != 0 || direction.latitude != 0);

    double cos;
    auto next = find_min_angle_neighbor(edge_id, direction, cos);

    // add edge with minimal angle
    if (cos >= max_angle_cos) [[unlikely]] {
        coordinate_t const destination_coordinate{_graph.node(next).coordinates};
        out.emplace_back(next, node.node(), node.distance());
        _destination_coordinates.emplace_back(destination_coordinate);
    }
}


//a rough approximation for the  function b^e for b = (1+eps)
template <typename B, typename E>
B pow_approximation(B const& base, E const& exponent) {
    return (exponent * (base - 1.0)) + 1.0;
}

//template <typename E>
//float pow_approximation(float const& base, E const& exponent) {
//    std::uint32_t _base = reinterpret_cast<std::uint32_t const&>(base);
//    std::uint32_t _exp_sign = reinterpret_cast<std::uint32_t const&>(base);
//
//    constexpr std::uint32_t mask_mantissa = 0x007FFFFF;
//    constexpr std::uint32_t mask_exp_sign = 0xFF800000;
//
//    _base &= mask_mantissa;
//    _exp_sign &= mask_exp_sign;
//
//    _base *= exponent;
//    _base |= _exp_sign;
//
//    return reinterpret_cast<float const&>(_base);
//}
//template <typename E>
//double pow_approximation(double const& base, E const& exponent) {
//    std::uint64_t _base = reinterpret_cast<std::uint64_t const&>(base);
//    std::uint64_t _exp_sign = reinterpret_cast<std::uint64_t const&>(base);
//
//    constexpr std::uint64_t mask_mantissa = 0x000FFFFFFFFFFFFF;
//    constexpr std::uint64_t mask_exp_sign = 0xFFF0000000000000;
//
//    _base &= mask_mantissa;
//    _exp_sign &= mask_exp_sign;
//
//    _base *= exponent;
//    _base |= _exp_sign;
//
//    return reinterpret_cast<double const&>(_base);
//}


template<typename Graph, typename Labels>
steiner_neighbors<Graph, Labels>::node_id_type
steiner_neighbors<Graph, Labels>::find_min_angle_neighbor(const base_edge_id_type&edge_id, const coordinate_t&direction, double&cos) {
    assert(direction.longitude != 0 || direction.latitude != 0);
    auto&&destination_steiner_info = _graph.steiner_info(edge_id);

    // binary search for node with minimal angle using the derivative over the angle depending on steiner index
    using intra_edge_id_type = typename node_id_type::intra_edge_id_type;
    intra_edge_id_type l = 1;
    intra_edge_id_type r = destination_steiner_info.node_count - 2;
    intra_edge_id_type m = destination_steiner_info.mid_index;
    double diff = 1.0;
    double cos2 = 1.0;

    steiner_graph::node_id_type destination{edge_id, m};
    steiner_graph::node_id_type destination_next{edge_id, m + 1};

    if (l >= r) [[unlikely]]
        return destination;

    coordinate_t direction_next;
    coordinate_t direction_current;
    // first check
    {
        // get coordinates
        direction_next = _graph.node(destination_next).coordinates;
        direction_current = _graph.node(destination).coordinates;

        // make directions from source
        assert(direction_next != _source_coordinate);
        assert(direction_current != _source_coordinate);
        direction_next -= _source_coordinate;
        direction_current -= _source_coordinate;

        // compute cos values (can hopefully be vectorized)
        diff = angle_cos(direction, direction_next);
        cos2 = angle_cos(direction, direction_current);
        assert(diff == 0 || std::isnormal(diff));
        assert(cos2 == 0 || std::isnormal(cos2));

        // keep difference of cosines
        diff -= cos2;
    }

    // depending on which half of the edge is used, compute a factor for selecting the next m-value
    bool const right_half = (diff >= 0.0);
    float const base = (right_half) ? destination_steiner_info.base_second : destination_steiner_info.base_first;
    float const log_base_inv = 1 / std::log(base);

    while (l < r && std::isnormal(diff)) [[likely]] {
        // update node ids
        destination_next.steiner_index = m + 1;
        destination.steiner_index = m;

        // get coordinates
        direction_next = _graph.node(destination_next).coordinates;
        direction_current = _graph.node(destination).coordinates;

        // make directions from source
        assert(direction_next != _source_coordinate);
        assert(direction_current != _source_coordinate);
        direction_next -= _source_coordinate;
        direction_current -= _source_coordinate;

        // compute cos values (can hopefully be vectorized)
        diff = angle_cos(direction, direction_next);
        cos2 = angle_cos(direction, direction_current);
        assert(diff == 0 || std::isnormal(diff));
        assert(cos2 == 0 || std::isnormal(cos2));

        // keep difference of cosines
        diff -= cos2;

        // update range
        bool right = (diff > 0.0);
        bool left = !right;
        l = right * (m + 1) + left * l;
        r = left * (m - 1) + right * r;

        // naive method for m-value : 8566117 iterations (for aegaeis-ref, 57200 -> 5146, eps = 0.2)
        // m = (l+r) / 2;

        // compute improved m-value, 6609402 iterations, can possibly be further improved
        intra_edge_id_type step = std::ceil((std::log(1 + pow_approximation(base, r - l)) - 1) * log_base_inv);
                    // go right
        m = right ? ((right_half)
                    ? (r - step)
                    : ((l + r) / 2))
                    // go left
                  : ((right_half)
                    ? ((l + r) / 2)
                    : (l + step));
        m = std::clamp(m, l, r);
        assert (l >= r || (l <= m && m <= r));

        _steiner_point_angle_test_count++;
    }
    assert(diff == 0 || std::isnormal(diff));
    assert(m >= 0 && m < destination_steiner_info.node_count);

    // store cos value
    cos = angle_cos(direction, direction_current);

    return destination;
}

template<typename Graph, typename Labels>
template<typename NodeCostPair>
void steiner_neighbors<Graph, Labels>::on_edge_neighbors(const NodeCostPair&node, std::vector<NodeCostPair>&out) {
    auto const&node_id = node.node();
    // auto const &reached_from = node.predecessor();
    auto&&steiner_info = _graph.steiner_info(node_id.edge);

    // for neighboring node on own edge
    if (node_id.steiner_index < steiner_info.node_count - 1) [[likely]] {
        steiner_graph::node_id_type const destination(node_id.edge, node_id.steiner_index + 1);
        // if (destination != reached_from) [[likely]] {
        coordinate_t destination_coordinate{_graph.node(destination).coordinates};
        assert(_graph.has_edge(node_id, destination));
        out.emplace_back(destination, node_id, node.distance());
        _destination_coordinates.emplace_back(destination_coordinate);
        // }
    }

    // for other neighboring node on own edge
    if (node_id.steiner_index > 0) [[likely]] {
        steiner_graph::node_id_type const destination(node_id.edge, node_id.steiner_index - 1);
        // if (destination != reached_from) [[likely]] {
        coordinate_t destination_coordinate{_graph.node(destination).coordinates};
        assert(_graph.has_edge(node_id, destination));
        out.emplace_back(destination, node_id, node.distance());
        _destination_coordinates.emplace_back(destination_coordinate);
        // }
    }
}

template<typename Graph, typename Labels>
bool steiner_neighbors<Graph, Labels>::ignore_edge(typename Graph::base_topology_type::edge_id_type const&edge_id,
                                                   coordinate_t const&direction) {
    return angle_cos(_graph.node(_graph.base_graph().source(edge_id)).coordinates - _source_coordinate, direction) <= 0.0
           && angle_cos(_graph.node(_graph.base_graph().destination(edge_id)).coordinates - _source_coordinate, direction) <= 0.0;
}
