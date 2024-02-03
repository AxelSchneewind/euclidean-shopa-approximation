#include "steiner_neighbors.h"
#include "../graph/base_types.h"

#include <cmath>
#include <vector>

template<typename Graph, typename Labels>
template<typename NodeCostPair>
typename Graph::node_id_type steiner_neighbors<Graph, Labels>::find_face_crossing_predecessor(const NodeCostPair &node) {
    if constexpr (HasFaceCrossingPredecessor<NodeCostPair, Graph>) {
        assert(!is_none(node.face_crossing_predecessor()));
        return node.face_crossing_predecessor();
    } else {
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
}

template<typename Graph, typename Labels>
template<typename NodeCostPair>
void steiner_neighbors<Graph, Labels>::operator()(const NodeCostPair &node, std::vector<NodeCostPair> &out) {
    auto const &node_id = node.node();
    assert(!is_none(node_id));

    _source_coordinate = _graph.node(node_id).coordinates;

    //
    bool is_start_node = node_id == node.predecessor();
    bool is_base_node = _graph.is_base_node(node_id);
    bool is_boundary_node = false;

    if constexpr (HasFaceCrossingPredecessor<NodeCostPair,Graph>) {
        assert(is_start_node || (!is_none(node.face_crossing_predecessor()) && node.face_crossing_predecessor() != node.node()));
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
        auto fcp = (is_none(node.face_crossing_predecessor()) || is_boundary_node) ?
                node.node() : node.face_crossing_predecessor();

        assert(!is_none(fcp));
        for (int e = 0; e < out.size(); ++e) [[likely]] {
            out[e].face_crossing_predecessor() = fcp;
        }

        for (int e = 0; e < out.size(); ++e) [[likely]] {
            out[e].face_crossing_predecessor() = fcp;
            if (out[e].face_crossing_predecessor() == out[e].node())
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
            _destination_coordinates.emplace_back(_graph.node(destination).coordinates);
        }
    }

    for (auto &&e: _graph.base_graph().incoming_edges(base_node_id)) [[likely]] {
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
void steiner_neighbors<Graph, Labels>::from_start_node(const NodeCostPair &node, std::vector<NodeCostPair> &out) {
    _base_node_count++;
    auto const &node_id = node.node();
    auto &&base_node_id = _graph.base_node_id(node_id);

    for (auto &&e: _graph.base_graph().outgoing_edges(base_node_id)) [[likely]] {
        auto e_id = _graph.base_graph().edge_id(base_node_id, e.destination);
        steiner_graph::node_id_type destination{e_id, 1};
        if (destination != node.predecessor()) [[likely]] {
            assert(_graph.has_edge(node_id, destination));
            out.emplace_back(destination, node_id, node.distance());
            _destination_coordinates.emplace_back(_graph.node(destination).coordinates);
        }
    }

    for (auto &&e: _graph.base_graph().incoming_edges(base_node_id)) [[likely]] {
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
        auto &&triangle_edges = _graph.base_polyhedron().node_edges(base_node_id);
        for (auto &&base_edge_id: triangle_edges) [[likely]] {
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
            _destination_coordinates.emplace_back(_graph.node(destination).coordinates);
        }
    }

    for (auto &&e: _graph.base_graph().incoming_edges(base_node_id)) [[likely]] {
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
        auto &&triangle_edges = _graph.base_polyhedron().node_edges(base_node_id);
        for (auto &&base_edge_id: triangle_edges) [[likely]] {
            epsilon_spanner(node, base_edge_id, -1.0, _source_coordinate, out);
        }
    }

    _boundary_node_neighbor_count += out.size();
}

template<typename Graph, typename Labels>
template<typename NodeCostPair>
void steiner_neighbors<Graph, Labels>::from_steiner_node(const NodeCostPair &node, std::vector<NodeCostPair> &out) {
    _steiner_point_count++;
    auto const &node_id = node.node();
    auto const &reached_from = node.predecessor();

    // always add the two neighbors on this edge
    on_edge_neighbors(node, out);

    auto&& face_crossing_predecessor = find_face_crossing_predecessor(node);
    coordinate_t direction = _source_coordinate - _graph.node(face_crossing_predecessor).coordinates;

    // normalize direction length so that it has the same length as the current edge (to prevent numerical issues)
    direction *= distance(_graph.node(_graph.base_graph().source(node.node().edge)).coordinates,
                     _graph.node(_graph.base_graph().destination(node.node().edge)).coordinates) / direction.length();

    assert(face_crossing_predecessor != node_id);
    assert(_graph.node(node_id).coordinates != _graph.node(face_crossing_predecessor).coordinates);
    assert(direction.latitude != 0.0 || direction.longitude != 0.0);

    // get triangles that have not been visited yet

    // make list of edges (i.e. destination/cost pairs)
    auto &&steiner_info = _graph.steiner_info(node_id.edge);

    // face-crossing edges
    auto &&triangles = _graph.base_polyhedron().edge_faces(node_id.edge);
    for (auto&& triangle : triangles) {
        if (is_none(triangle)) continue;
        assert(!is_none(triangle));

        auto &&triangle_edges = _graph.base_polyhedron().face_edges(triangle);
        for (auto &&base_edge_id: triangle_edges) [[likely]] {
            if (base_edge_id == node_id.edge/* || ignore_edge(node, base_edge_id, direction)*/) [[unlikely]]
                continue;

            add_min_angle_neighbor(node, base_edge_id, _max_angle_cos, direction, out);

            // required in paper, but probably not on unweighted triangulations
            // if (face_crossing_predecessor != reached_from && _source_coordinate != _graph.node(reached_from).coordinates) [[unlikely]] {
            //     assert(_graph.node(reached_from).coordinates != _source_coordinate);
            //     add_min_angle_neighbor(node, base_edge_id, _max_angle_cos, _source_coordinate - _graph.node(reached_from).coordinates, out);
            // }
        }
    }


    _steiner_point_neighbor_count += out.size();
}

template<typename Graph, typename Labels>
template<typename NodeCostPair>
void steiner_neighbors<Graph, Labels>::epsilon_spanner(const NodeCostPair &node, const steiner_neighbors<Graph, Labels>::base_edge_id_type &edge_id,
                                                       const double &max_angle_cos, const coordinate_t &direction,
                                                       std::vector<NodeCostPair> &out) {
    auto const &node_id = node.node();
    auto &&destination_steiner_info = _graph.steiner_info(edge_id);

    node_id_type next {edge_id, destination_steiner_info.mid_index};
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
inline steiner_neighbors<Graph, Labels>::node_id_type
steiner_neighbors<Graph, Labels>::add_min_angle_neighbor(const NodeCostPair &node, const steiner_neighbors<Graph, Labels>::base_edge_id_type &edge_id,
                                                         const double &max_angle_cos, const coordinate_t &direction,
                                                         std::vector<NodeCostPair> &out) {
    assert(direction.longitude != 0 || direction.latitude != 0);

    auto next = find_min_angle_neighbor<NodeCostPair>(edge_id, direction);

    // add edge with minimal angle
    coordinate_t const destination_coordinate{_graph.node(next).coordinates};
    out.emplace_back(next, node.node(), node.distance());
    _destination_coordinates.emplace_back(destination_coordinate);
    return next;
}

template<typename Graph, typename Labels>
template<typename NodeCostPair>
steiner_neighbors<Graph, Labels>::node_id_type
steiner_neighbors<Graph, Labels>::find_min_angle_neighbor(const steiner_neighbors<Graph, Labels>::base_edge_id_type &edge_id,
                                                          const coordinate_t &direction) {
    assert(direction.longitude != 0 || direction.latitude != 0);
    auto &&destination_steiner_info = _graph.steiner_info(edge_id);

    // binary search for node with minimal angle using the derivative over the angle depending on steiner index
    typename node_id_type::intra_edge_id_type l = 1;
    typename node_id_type::intra_edge_id_type r = destination_steiner_info.node_count - 2;
    typename node_id_type::intra_edge_id_type m = (r + l) / 2;
    double diff = 1.0;
    double cos2 = 1.0;

    steiner_graph::node_id_type destination{edge_id, m};
    steiner_graph::node_id_type destination_next{edge_id, m + 1};
    while (l < r && std::isnormal(diff)) [[likely]] {
        // get coordinates
        coordinate_t direction_next = _graph.node(destination_next).coordinates;
        coordinate_t direction_current = _graph.node(destination).coordinates;

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
        int right = (diff > 0.0);
        int left = !right;
        l = right * (m + 1) + left * l;
        r = left * (m - 1) + right * r;
        m = (l + r) / 2;

        // update node ids
        destination_next.steiner_index = m + 1;
        destination.steiner_index = m;
    }
    assert(diff == 0 || std::isnormal(diff));

    return destination;
}

template<typename Graph, typename Labels>
template<typename NodeCostPair>
void steiner_neighbors<Graph, Labels>::on_edge_neighbors(const NodeCostPair &node, std::vector<NodeCostPair> &out) {
    auto const &node_id = node.node();
    // auto const &reached_from = node.predecessor();
    auto &&steiner_info = _graph.steiner_info(node_id.edge);

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
template<typename NodeCostPair>
bool steiner_neighbors<Graph, Labels>::ignore_edge(const NodeCostPair &node, typename Graph::base_topology_type::edge_id_type const&edge_id, coordinate_t const& direction) {
    return angle_cos(_graph.node(_graph.base_graph().source(edge_id)).coordinates, direction) < 0.0
            && angle_cos(_graph.node(_graph.base_graph().destination(edge_id)).coordinates, direction) < 0.0;
}
